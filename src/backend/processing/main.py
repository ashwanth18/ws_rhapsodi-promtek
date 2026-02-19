import json
import os
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import re

import pandas as pd
import requests
from fastapi import FastAPI, HTTPException
from mcap_ros2.reader import read_ros2_messages
from pydantic import BaseModel


BACKEND_URL = os.environ.get('BACKEND_URL', 'http://backend:8000')
PHASE_TOPIC = os.environ.get('PHASE_TOPIC', '/lightsout_training/phase')
RUN_ID_TOPIC = os.environ.get('RUN_ID_TOPIC', '/lightsout_training/run_id')
BATCH_ID_TOPIC = os.environ.get(
    'BATCH_ID_TOPIC', '/lightsout_training/batch_id'
)
INGREDIENT_ID_TOPIC = os.environ.get(
    'INGREDIENT_ID_TOPIC', '/lightsout_training/ingredient_id'
)
TARGET_WEIGHT_TOPIC = os.environ.get(
    'TARGET_WEIGHT_TOPIC', '/lightsout_training/target_weight_g'
)
MODE_TOPIC = os.environ.get('MODE_TOPIC', '/lightsout_training/mode')
EPISODE_TOPIC = os.environ.get(
    'EPISODE_TOPIC', '/lightsout_training/episode'
)
ROBOT_ID_TOPIC = os.environ.get(
    'ROBOT_ID_TOPIC', '/lightsout_training/robot_id'
)
TOPICS_FILTER = [
    '/weight',
    PHASE_TOPIC,
    RUN_ID_TOPIC,
    BATCH_ID_TOPIC,
    INGREDIENT_ID_TOPIC,
    TARGET_WEIGHT_TOPIC,
    MODE_TOPIC,
    EPISODE_TOPIC,
    ROBOT_ID_TOPIC,
]
PHASE_START = os.environ.get('PHASE_START', 'pour_start')
PHASE_END = os.environ.get('PHASE_END', 'pour_end')
SCOOP_START = os.environ.get('SCOOP_START', 'scoop_start')
SCOOP_END = os.environ.get('SCOOP_END', 'scoop_end')
SETTLE_TOL_G = float(os.environ.get('SETTLE_TOL_G', '0.5'))
SETTLE_WINDOW_S = float(os.environ.get('SETTLE_WINDOW_S', '1.0'))
PATH_MAP_FROM = os.environ.get('PATH_MAP_FROM', '').strip()
PATH_MAP_TO = os.environ.get('PATH_MAP_TO', '').strip()


class ProcessRequest(BaseModel):
    run_db_id: Optional[int] = None
    run_folder: Optional[str] = None
    bag_path: Optional[str] = None
    out_path: Optional[str] = None


@dataclass
class WeightSample:
    t_ns: int
    weight_g: float


@dataclass
class PhaseEvent:
    t_ns: int
    phase: str


@dataclass
class MetaValue:
    t_ns: int
    value: str


app = FastAPI(title='Rhapsodi Processing Service')


def _find_mcap_file(bag_path: Path) -> Path:
    if bag_path.is_file():
        return bag_path
    if not bag_path.exists():
        raise FileNotFoundError(f'Bag path not found: {bag_path}')
    mcap_files = sorted(bag_path.glob('*.mcap'))
    if not mcap_files:
        raise FileNotFoundError(f'No .mcap files found in {bag_path}')
    return mcap_files[0]


def _remap_path(path: Path) -> Path:
    if not PATH_MAP_FROM or not PATH_MAP_TO:
        return path
    try:
        rel = path.resolve().relative_to(Path(PATH_MAP_FROM).resolve())
    except ValueError:
        return path
    return Path(PATH_MAP_TO) / rel


def _load_metadata(run_folder: Path) -> Tuple[Dict[str, Any], Path]:
    metadata_path = run_folder / 'metadata.json'
    if not metadata_path.exists():
        raise FileNotFoundError(f'metadata.json not found in {run_folder}')
    metadata_json = metadata_path.read_text()
    metadata = json.loads(metadata_json or '{}')
    bag_path = Path(metadata.get('bag_path') or (run_folder / 'data'))
    if not bag_path.exists():
        bag_path = _remap_path(bag_path)
    return metadata, bag_path


def _read_samples(
    bag_path: Path,
) -> Tuple[Path, List[WeightSample], List[PhaseEvent], Dict[str, MetaValue]]:
    mcap_path = _find_mcap_file(bag_path)
    weights: List[WeightSample] = []
    phases: List[PhaseEvent] = []
    meta: Dict[str, MetaValue] = {}
    for msg in read_ros2_messages(mcap_path, topics=TOPICS_FILTER):
        topic = msg.channel.topic
        log_time = msg.log_time
        if isinstance(log_time, datetime):
            log_time_ns = int(log_time.timestamp() * 1e9)
        else:
            log_time_ns = int(log_time)
        if topic == '/weight':
            weights.append(
                WeightSample(
                    t_ns=log_time_ns, weight_g=float(msg.ros_msg.data)
                )
            )
        elif topic == PHASE_TOPIC:
            phases.append(
                PhaseEvent(t_ns=log_time_ns, phase=str(msg.ros_msg.data))
            )
        elif topic in {
            RUN_ID_TOPIC,
            BATCH_ID_TOPIC,
            INGREDIENT_ID_TOPIC,
            MODE_TOPIC,
            ROBOT_ID_TOPIC,
        }:
            meta[topic] = MetaValue(log_time_ns, str(msg.ros_msg.data))
        elif topic == TARGET_WEIGHT_TOPIC:
            meta[topic] = MetaValue(log_time_ns, str(float(msg.ros_msg.data)))
        elif topic == EPISODE_TOPIC:
            if topic not in meta:
                meta[topic] = MetaValue(
                    log_time_ns, str(int(msg.ros_msg.data))
                )
    weights.sort(key=lambda x: x.t_ns)
    phases.sort(key=lambda x: x.t_ns)
    return mcap_path, weights, phases, meta


def _nearest_weight(weights: List[WeightSample], t_ns: int) -> Optional[float]:
    if not weights:
        return None
    for w in weights:
        if w.t_ns >= t_ns:
            return w.weight_g
    return weights[-1].weight_g


def _weight_at_or_before(
    weights: List[WeightSample], t_ns: int
) -> Optional[float]:
    if not weights:
        return None
    prev = None
    for w in weights:
        if w.t_ns > t_ns:
            break
        prev = w.weight_g
    return prev if prev is not None else weights[0].weight_g


def _meta_value(meta: Dict[str, MetaValue], topic: str) -> Optional[str]:
    if topic not in meta:
        return None
    return meta[topic].value


def _episode_from_path(mcap_path: Path) -> Optional[int]:
    candidates = [mcap_path.stem, mcap_path.parent.name]
    for name in candidates:
        match = re.search(r'episode_(\d+)', name)
        if match:
            try:
                return int(match.group(1))
            except ValueError:
                return None
    return None


def _baseline_weight(
    weights: List[WeightSample], pour_start_ns: Optional[int]
) -> Optional[float]:
    if not weights:
        return None
    if pour_start_ns is None:
        window = weights[: min(10, len(weights))]
        return sum(w.weight_g for w in window) / len(window)
    pre = [w.weight_g for w in weights if w.t_ns < pour_start_ns]
    if not pre:
        return weights[0].weight_g
    window = pre[-min(10, len(pre)):]
    return sum(window) / len(window)


def _settle_time_s(
    weights: List[WeightSample],
    pour_end_ns: Optional[int],
    final_weight: float,
) -> Optional[float]:
    if pour_end_ns is None or not weights:
        return None
    window_ns = int(SETTLE_WINDOW_S * 1e9)
    idx = 0
    while idx < len(weights) and weights[idx].t_ns < pour_end_ns:
        idx += 1
    for i in range(idx, len(weights)):
        t_start = weights[i].t_ns
        t_end = t_start + window_ns
        ok = True
        for w in weights[i:]:
            if w.t_ns > t_end:
                break
            if abs(w.weight_g - final_weight) > SETTLE_TOL_G:
                ok = False
                break
        if ok:
            return (t_start - pour_end_ns) / 1e9
    return None


def _compute_features(
    weights: List[WeightSample],
    phases: List[PhaseEvent],
    meta: Dict[str, MetaValue],
) -> Dict[str, Any]:
    target_weight = _meta_value(meta, TARGET_WEIGHT_TOPIC)
    target_weight = float(target_weight) if target_weight is not None else None

    if weights:
        total_episode_time_s = (weights[-1].t_ns - weights[0].t_ns) / 1e9
    else:
        total_episode_time_s = None

    pour_start_ns = next(
        (p.t_ns for p in phases if p.phase == PHASE_START), None
    )
    pour_end_ns = next(
        (p.t_ns for p in phases if p.phase == PHASE_END), None
    )
    scoop_start_ns = next(
        (p.t_ns for p in phases if p.phase == SCOOP_START), None
    )
    scoop_end_ns = next(
        (p.t_ns for p in phases if p.phase == SCOOP_END), None
    )
    pour_duration_s = (
        (pour_end_ns - pour_start_ns) / 1e9
        if pour_start_ns and pour_end_ns
        else None
    )
    scoop_duration_s = (
        (scoop_end_ns - scoop_start_ns) / 1e9
        if scoop_start_ns and scoop_end_ns
        else None
    )

    baseline = _baseline_weight(weights, pour_start_ns)
    pour_weights: List[WeightSample] = []
    if pour_start_ns and pour_end_ns:
        pour_weights = [
            w for w in weights if pour_start_ns <= w.t_ns <= pour_end_ns
        ]
    final_weight = (
        _weight_at_or_before(weights, pour_end_ns)
        if pour_end_ns is not None
        else (weights[-1].weight_g if weights else None)
    )
    max_weight = (
        max((w.weight_g for w in pour_weights), default=None)
        if pour_weights
        else max((w.weight_g for w in weights), default=None)
    )
    net_weight = (
        final_weight - baseline
        if final_weight is not None and baseline is not None
        else None
    )
    overshoot = (
        max(0.0, max_weight - target_weight)
        if max_weight is not None and target_weight is not None
        else None
    )

    avg_flow = None
    if (
        pour_start_ns
        and pour_end_ns
        and pour_duration_s
        and pour_duration_s > 0
    ):
        w_start = _nearest_weight(weights, pour_start_ns)
        w_end = _nearest_weight(weights, pour_end_ns)
        if w_start is not None and w_end is not None:
            avg_flow = (w_end - w_start) / pour_duration_s

    settle_time = (
        _settle_time_s(weights, pour_end_ns, final_weight)
        if final_weight is not None
        else None
    )

    start_time_ns = None
    end_time_ns = None
    timestamps = [w.t_ns for w in weights] + [p.t_ns for p in phases]
    if timestamps:
        start_time_ns = min(timestamps)
        end_time_ns = max(timestamps)

    return {
        'run_id': _meta_value(meta, RUN_ID_TOPIC),
        'robot_id': _meta_value(meta, ROBOT_ID_TOPIC),
        'batch_id': _meta_value(meta, BATCH_ID_TOPIC),
        'ingredient_id': _meta_value(meta, INGREDIENT_ID_TOPIC),
        'mode': _meta_value(meta, MODE_TOPIC),
        'episode_index': (
            int(_meta_value(meta, EPISODE_TOPIC))
            if _meta_value(meta, EPISODE_TOPIC)
            else None
        ),
        'target_weight_g': target_weight,
        'baseline_weight_g': baseline,
        'final_weight_g': final_weight,
        'net_weight_g': net_weight,
        'max_weight_g': max_weight,
        'overshoot_g': overshoot,
        'avg_flow_rate_g_s': avg_flow,
        'total_episode_time_s': total_episode_time_s,
        'pour_duration_s': pour_duration_s,
        'scoop_duration_s': scoop_duration_s,
        'settle_time_s': settle_time,
        'start_time_ns': start_time_ns,
        'end_time_ns': end_time_ns,
        'phase_events_json': json.dumps(
            [{'t_ns': p.t_ns, 'phase': p.phase} for p in phases]
        ),
    }


def _write_parquet(
    weights: List[WeightSample],
    phases: List[PhaseEvent],
    meta: Dict[str, MetaValue],
    out_path: Path,
) -> Optional[str]:
    if not weights and not phases:
        return None
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, Any]] = []
    run_id = _meta_value(meta, RUN_ID_TOPIC)
    batch_id = _meta_value(meta, BATCH_ID_TOPIC)
    ingredient_id = _meta_value(meta, INGREDIENT_ID_TOPIC)
    mode = _meta_value(meta, MODE_TOPIC)
    robot_id = _meta_value(meta, ROBOT_ID_TOPIC)
    episode_index = _meta_value(meta, EPISODE_TOPIC)
    target_weight = _meta_value(meta, TARGET_WEIGHT_TOPIC)

    for w in weights:
        rows.append(
            {
                'log_time_ns': w.t_ns,
                'weight_g': w.weight_g,
                'phase': None,
                'run_id': run_id,
                'batch_id': batch_id,
                'ingredient_id': ingredient_id,
                'mode': mode,
                'robot_id': robot_id,
                'episode_index': episode_index,
                'target_weight_g': target_weight,
            }
        )
    for p in phases:
        rows.append(
            {
                'log_time_ns': p.t_ns,
                'weight_g': None,
                'phase': p.phase,
                'run_id': run_id,
                'batch_id': batch_id,
                'ingredient_id': ingredient_id,
                'mode': mode,
                'robot_id': robot_id,
                'episode_index': episode_index,
                'target_weight_g': target_weight,
            }
        )
    df = pd.DataFrame(rows).sort_values('log_time_ns')
    df.to_parquet(out_path, index=False)
    return str(out_path)


@app.post('/process')
def process(req: ProcessRequest) -> Dict[str, Any]:
    try:
        if not req.bag_path:
            raise ValueError('bag_path required')
        bag_path = Path(req.bag_path).expanduser().resolve()
        if not bag_path.exists():
            bag_path = _remap_path(bag_path)
        mcap_path, weights, phases, meta = _read_samples(bag_path)
        episode_from_path = _episode_from_path(mcap_path)
        if episode_from_path is not None:
            try:
                meta_episode = (
                    int(_meta_value(meta, EPISODE_TOPIC))
                    if _meta_value(meta, EPISODE_TOPIC)
                    else None
                )
            except ValueError:
                meta_episode = None
            if meta_episode is None or meta_episode != episode_from_path:
                meta[EPISODE_TOPIC] = MetaValue(
                    0, str(episode_from_path)
                )
        features = _compute_features(weights, phases, meta)
        out_path = (
            Path(req.out_path)
            if req.out_path
            else mcap_path.with_suffix('.parquet')
        )
        parquet_path = _write_parquet(weights, phases, meta, out_path)
        features['parquet_path'] = parquet_path
        features['features_json'] = json.dumps(
            {
                'weights_count': len(weights),
                'phases_count': len(phases),
            }
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    payload = {'run_db_id': req.run_db_id, **features}
    try:
        resp = requests.post(
            f'{BACKEND_URL}/processed', json=payload, timeout=10
        )
        resp.raise_for_status()
    except Exception as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return resp.json()
