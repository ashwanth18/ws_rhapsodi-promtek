"""Heuristic planner vs authored/random shifts in ``ScoopEnv``, and a ``fill_efficiency`` fit.

``fill_efficiency`` (scoop_vision ``planner``) is scooped volume over the
powder volume the swept scoop engages. The fit is the ratio of totals over the
heuristic's executed scoops; the heuristic is then re-run with it.

    ros2 run isaac_twin scoop_env_compare --episodes 2 --max-scoops 12
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from isaac_twin.gym.scoop_env import ScoopEnv


def run_policy(env: ScoopEnv, policy: str, episodes: int, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    scooped, errors, violations, steps = [], [], 0, 0
    engaged_m3 = scooped_m3 = 0.0
    for ep in range(episodes):
        env.reset(seed=seed + ep)
        done = False
        while not done:
            plan = None
            if policy == "heuristic":
                plan = env.heuristic_plan()
                if not plan.success:
                    break
                action = np.array([plan.offset_x, plan.offset_y, plan.offset_z])
            elif policy == "authored":
                action = np.zeros(3)
            else:
                action = rng.uniform(env.action_space.low, env.action_space.high)
            _obs, _r, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            steps += 1
            if not info["executed"]:
                violations += 1
                continue
            scooped.append(info["scooped_g"])
            errors.append(abs(info["scooped_g"] - env.target_g) / env.target_g)
            if plan is not None:
                engaged_m3 += plan.engaged_volume_m3
                scooped_m3 += info["scooped_g"] / (1e6 * env.powder.params.density_g_per_ml)
    out = {
        "policy": policy,
        "steps": steps,
        "executed": len(scooped),
        "violations": violations,
        "mean_scooped_g": float(np.mean(scooped)) if scooped else 0.0,
        "std_scooped_g": float(np.std(scooped)) if scooped else 0.0,
        "mean_abs_error": float(np.mean(errors)) if errors else 1.0,
    }
    if engaged_m3 > 0:
        out["fitted_fill_efficiency"] = scooped_m3 / engaged_m3
    return out


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--layout", default="dual-container")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--max-scoops", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policies", default="heuristic,authored,random")
    ap.add_argument("--no-reachability", action="store_true", help="Skip the IK check (no URDF needed)")
    ap.add_argument("--json", default="", help="Write the results here")
    args = ap.parse_args(argv)

    env = ScoopEnv(layout_id=args.layout, max_scoops=args.max_scoops, check_reachability=not args.no_reachability)
    print(f"target {env.target_g:.1f} g (capacity {env.capacity_g:.1f} g), planner fill_efficiency "
          f"{env.planner.params.fill_efficiency:.2f}, reachability {'on' if env.chain else 'off'}")
    results = []
    for policy in args.policies.split(","):
        results.append(run_policy(env, policy.strip(), args.episodes, args.seed))
        print(json.dumps(results[-1]))

    fit = next((r["fitted_fill_efficiency"] for r in results if "fitted_fill_efficiency" in r), None)
    if fit is not None:
        env.planner.params.fill_efficiency = float(np.clip(fit, 0.05, 1.0))
        refit = run_policy(env, "heuristic", args.episodes, args.seed)
        refit["policy"] = f"heuristic(fill_efficiency={env.planner.params.fill_efficiency:.3f})"
        results.append(refit)
        print(json.dumps(refit))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"target_g": env.target_g, "results": results}, fh, indent=2)


if __name__ == "__main__":
    main()
