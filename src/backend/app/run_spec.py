"""Canonical run identity / job shape shared by every operating mode.

Every weighment or training episode — MES Condor, MES generic, mock-local,
or lights-out — is described by one ``RunSpec``. Downstream systems
(orchestrator start services, data_collection_manager metadata.json,
backend ``runs`` rows) should derive their fields from this contract
rather than inventing per-mode shapes.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, model_validator


class OperatingMode(str, Enum):
    MES_CONDOR = 'mes-condor'
    MES_GENERIC = 'mes-generic'
    MOCK_LOCAL = 'mock-local'
    LIGHTSOUT = 'lightsout'


class Environment(str, Enum):
    REAL = 'real'
    SIM = 'sim'


# Behavior tree / start-service identity. MES family + mock share one tree;
# lights-out uses a separate tree.
TREE_WEBHOOK_WEIGHTMENT = 'WebhookWeightment'
TREE_LIGHTSOUT = 'LightsOut'

_MES_FAMILY = frozenset(
    {
        OperatingMode.MES_CONDOR,
        OperatingMode.MES_GENERIC,
        OperatingMode.MOCK_LOCAL,
    }
)


class RunLabel(BaseModel):
    """Snapshot of powder / operator labels written into each run.

    Stored by value (not catalog FK) so renaming a powder later cannot
    rewrite historical training data.
    """

    powder_id: Optional[str] = None
    powder_name: Optional[str] = None
    container_target: Optional[str] = None
    pour_target: Optional[str] = None
    lot_code: Optional[str] = None
    operator: Optional[str] = None
    notes: Optional[str] = None
    cycles: Optional[int] = None
    stop_on: Optional[str] = None
    stop_value: Optional[float] = None
    target_mode: Optional[str] = None
    target_fractions: Optional[List[float]] = None
    rng_seed: Optional[int] = None


class RunSpec(BaseModel):
    mode: OperatingMode
    environment: Environment = Environment.REAL
    run_key: str
    target_weight_g: float
    tolerance_g: float
    location_code: Optional[str] = None
    location_id: Optional[str] = None
    batch_id: Optional[str] = None
    ingredient_id: Optional[str] = None
    weightment_id: Optional[str] = None
    expected_lot: Optional[str] = None
    tree_id: str = ''
    label: Optional[RunLabel] = None

    @model_validator(mode='after')
    def _default_tree_id(self) -> 'RunSpec':
        if not self.tree_id:
            self.tree_id = (
                TREE_LIGHTSOUT
                if self.mode == OperatingMode.LIGHTSOUT
                else TREE_WEBHOOK_WEIGHTMENT
            )
        return self

    @property
    def phase_topic(self) -> str:
        if self.mode == OperatingMode.LIGHTSOUT:
            return '/lightsout_training/phase'
        return '/webhook_run/phase'

    @property
    def is_mes_family(self) -> bool:
        return self.mode in _MES_FAMILY

    def to_metadata_dict(self) -> Dict[str, Any]:
        """Flat dict suitable for merging into run ``metadata.json``."""
        payload: Dict[str, Any] = {
            'schema_version': '1',
            'mode': self.mode.value,
            'environment': self.environment.value,
            'run_key': self.run_key,
            'target_weight_g': self.target_weight_g,
            'tolerance_g': self.tolerance_g,
            'tree_id': self.tree_id,
            'phase_topic': self.phase_topic,
        }
        optional = {
            'location_code': self.location_code,
            'location_id': self.location_id,
            'batch_id': self.batch_id,
            'ingredient_id': self.ingredient_id,
            'weightment_id': self.weightment_id,
            'expected_lot': self.expected_lot,
        }
        for key, value in optional.items():
            if value is not None:
                payload[key] = value
        if self.label is not None:
            label_payload = self.label.model_dump(exclude_none=True)
            payload['label'] = label_payload
            # Also flatten common label keys for consumers that read top-level.
            for key in (
                'powder_id',
                'powder_name',
                'container_target',
                'pour_target',
                'lot_code',
                'operator',
                'notes',
                'cycles',
                'stop_on',
                'stop_value',
                'target_mode',
                'rng_seed',
            ):
                if key in label_payload and key not in payload:
                    payload[key] = label_payload[key]
        return payload
