"""DEPRECATED: local buildx runner.

Builds are now triggered via GitHub Actions workflow_dispatch
(see app/github.py::trigger_workflow). This module is kept only so any
old import sites fail loudly if reintroduced.
"""
from __future__ import annotations


def run_branch_build(*_args, **_kwargs):  # noqa: ANN001
    raise RuntimeError(
        'Local build_runner is retired. Use github.trigger_workflow(branch) '
        'so CI reports a verified Release to POST /api/releases/report.'
    )
