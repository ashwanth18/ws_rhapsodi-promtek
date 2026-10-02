"""Gymnasium envs over the twin's cell model (``gymnasium.make("IsaacTwin/Scoop-v0")``)."""

from gymnasium.envs.registration import register

from isaac_twin.gym.scoop_env import ScoopEnv

register(id="IsaacTwin/Scoop-v0", entry_point="isaac_twin.gym.scoop_env:ScoopEnv")

__all__ = ["ScoopEnv"]
