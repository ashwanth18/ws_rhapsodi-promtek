"""Semi-automated knowledge-base drafting job (self-updating-kb).

Reads every incident from the ingestion database (occurrence stats are
always rendered from the full history, not just incidents seen since
the last run - see `run()`'s docstring for why) and drafts updates to
two files that live in the main repo (not this service's own tree):

- ``.cursor/rules/robot-fault-patterns.mdc``: machine-readable,
  glob-attached Cursor rule so a future Cursor session automatically has
  known failure signatures + fixes in context.
- ``TROUBLESHOOTING.md``: the generalized, all-subsystem successor to
  ``src/backend/webhook-run-observability.md``'s postmortem log.

"Drafts, not auto-commits" (per the plan): this script only writes
files to the working tree - it never runs ``git add``/``commit``/
``push``. Reviewing the resulting diff and committing it is a human
(or reviewing-agent) step, matching the plan's "semi-automated" choice.
Meant to be run periodically/on-demand (e.g. via the ``/loop`` skill),
not as an always-on service.

Each output file is split into ``<!-- kb:begin:{signature_id} -->`` /
``<!-- kb:end:{signature_id} -->`` marked sections, one per signature.
Re-running this job only replaces those marked sections (fresh
occurrence counts/timestamps) or appends a new one for a signature seen
for the first time - any other hand-written content in the file (title,
intro, unrelated sections) is left exactly as a human left it.

Incidents are matched to their prose via ``ingestion.kb_signatures`` -
a signature detected by ``ingestion.rules`` with no corresponding
``SignatureKnowledge`` entry is skipped with a warning rather than
drafting an empty/uninformative section (a rules-engine change should
come with a matching knowledge entry, not silently produce a stub).
"""
from __future__ import annotations

import argparse
import logging
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from ingestion.database import SessionLocal
from ingestion.kb_signatures import SIGNATURES, SignatureKnowledge, resolve_signature_id
from ingestion.models import Incident

logger = logging.getLogger('ingestion.kb_draft_job')

DEFAULT_RULE_RELATIVE_PATH = Path('.cursor/rules/robot-fault-patterns.mdc')
DEFAULT_TROUBLESHOOTING_RELATIVE_PATH = Path('TROUBLESHOOTING.md')

_BEGIN_MARKER = '<!-- kb:begin:{sig} -->'
_END_MARKER = '<!-- kb:end:{sig} -->'
_SEVERITY_ORDER = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'CRITICAL']

_RULE_GLOBS = ','.join(
    [
        'src/backend/**/*.py',
        'src/data_collection_manager/**/*.py',
        'src/rhapsodi_common/**/*.py',
        'src/rhapsodi_common_cpp/**/*',
        'src/pouring_controller/**/*',
        'src/weighing_scale_driver/**/*',
        'src/robot_orchestrator/**/*',
    ]
)

_RULE_HEADER = f"""---
description: Known robot/backend failure signatures and their fixes, auto-drafted from the incidents table by ingestion/kb_draft_job.py (self-updating-kb) - review before trusting blindly, especially occurrence counts.
globs: {_RULE_GLOBS}
alwaysApply: false
---

# Robot fault patterns

Known failure signatures detected by `central-ingestion-service`'s
rules engine (`src/backend/ingestion/ingestion/rules.py`) plus a few
still-undetected legacy symptoms, so a Cursor session working on any of
the subsystems above already has these in context. Each section below
is auto-generated from `src/backend/ingestion/ingestion/kb_signatures.py`
and real `incidents` table occurrences - **do not hand-edit the marked
sections**, they get overwritten on the next drafting pass; edit
`kb_signatures.py` instead and re-run
`python -m ingestion.kb_draft_job`.
"""

_TROUBLESHOOTING_HEADER = """# Troubleshooting

Generalized, all-subsystem successor to
`src/backend/webhook-run-observability.md`'s postmortem log. Sections
below are auto-drafted from the `incidents` table by
`src/backend/ingestion/ingestion/kb_draft_job.py`
(self-updating-kb) - **do not hand-edit the marked sections**, they get
overwritten on the next drafting pass; edit
`src/backend/ingestion/ingestion/kb_signatures.py` instead and re-run
`python -m ingestion.kb_draft_job`. Add hand-written context anywhere
outside the marked sections and it will be preserved.
"""


@dataclass
class SignatureOccurrence:
    signature_id: str
    count: int = 0
    max_severity: Optional[str] = None
    first_seen: Optional[datetime] = None
    last_seen: Optional[datetime] = None
    device_ids: List[str] = field(default_factory=list)
    run_keys: List[str] = field(default_factory=list)

    def absorb(self, incident: Incident) -> None:
        self.count += 1
        if _severity_rank(incident.severity) > _severity_rank(self.max_severity):
            self.max_severity = incident.severity
        detected_at = incident.detected_at
        if detected_at is not None:
            if self.first_seen is None or detected_at < self.first_seen:
                self.first_seen = detected_at
            if self.last_seen is None or detected_at > self.last_seen:
                self.last_seen = detected_at
        if incident.device_id and incident.device_id not in self.device_ids:
            self.device_ids.append(incident.device_id)
        if incident.run_key and incident.run_key not in self.run_keys:
            self.run_keys.append(incident.run_key)


def _severity_rank(severity: Optional[str]) -> int:
    try:
        return _SEVERITY_ORDER.index((severity or '').upper())
    except ValueError:
        return -1


def summarize_incidents(
    incidents: Iterable[Incident],
) -> Dict[str, SignatureOccurrence]:
    """Groups raw `Incident` rows into one occurrence summary per
    KB-resolved signature (see `kb_signatures.resolve_signature_id` -
    e.g. `pour_overshoot_severe` folds into `pour_overshoot`'s summary).
    """
    summaries: Dict[str, SignatureOccurrence] = {}
    for incident in incidents:
        key = resolve_signature_id(incident.signature_id)
        summaries.setdefault(key, SignatureOccurrence(signature_id=key)).absorb(
            incident
        )
    return summaries


def _fmt_timestamp(value: Optional[datetime]) -> str:
    if value is None:
        return 'unknown'
    return value.strftime('%Y-%m-%d %H:%M UTC')


def render_signature_section(
    knowledge: SignatureKnowledge, occurrence: Optional[SignatureOccurrence]
) -> str:
    lines = [_BEGIN_MARKER.format(sig=knowledge.signature_id)]
    lines.append(f'## {knowledge.title} (`{knowledge.signature_id}`)')
    lines.append('')
    lines.append(f'- **Category:** {knowledge.category}')
    lines.append(f'- **Symptom:** {knowledge.symptom}')
    lines.append(f'- **Root cause:** {knowledge.root_cause}')
    lines.append(f'- **Fix:** {knowledge.fix}')
    lines.append('- **If this appears again, check:**')
    for item in knowledge.if_recurs:
        lines.append(f'  - {item}')
    if occurrence and occurrence.count > 0:
        devices = ', '.join(occurrence.device_ids) or 'unknown'
        lines.append(
            f'- _Observed {occurrence.count}x in incidents, most '
            f'recently {_fmt_timestamp(occurrence.last_seen)} (max '
            f'severity {occurrence.max_severity}) on device(s): '
            f'{devices}._'
        )
    else:
        lines.append(
            '- _Not yet observed in a recorded incident - documented '
            'pre-emptively from the known code path._'
        )
    lines.append(_END_MARKER.format(sig=knowledge.signature_id))
    return '\n'.join(lines)


def _section_pattern(signature_id: str) -> re.Pattern:
    begin = re.escape(_BEGIN_MARKER.format(sig=signature_id))
    end = re.escape(_END_MARKER.format(sig=signature_id))
    return re.compile(f'{begin}.*?{end}', re.DOTALL)


def upsert_sections(
    existing_content: str, header: str, sections: Dict[str, str]
) -> str:
    """Replaces each signature's marked section in-place if present,
    appends it at the end if this is a signature seen for the first
    time, and otherwise leaves `existing_content` completely untouched -
    any hand-written prose outside the markers survives every re-run.
    A brand-new file starts from `header` plus every section.
    """
    content = existing_content if existing_content.strip() else header
    for signature_id, rendered in sections.items():
        pattern = _section_pattern(signature_id)
        if pattern.search(content):
            # A replacement function (not a string) is used deliberately:
            # `re.sub` interprets backslash-escapes like `\1`/`\g<...>` in
            # string replacements, which would corrupt rendered prose
            # that happens to contain a literal backslash.
            content = pattern.sub(lambda _m, r=rendered: r, content)
        else:
            separator = '\n\n' if not content.endswith('\n\n') else ''
            content = f'{content.rstrip()}\n{separator}{rendered}\n'
    return content


def draft_kb(
    incidents: List[Incident],
    existing_rule_content: str = '',
    existing_troubleshooting_content: str = '',
) -> tuple:
    """Pure rendering step (no filesystem/DB I/O) - returns the updated
    `(rule_content, troubleshooting_content, drafted_incident_ids)`.
    Kept separate from `run()` so the rendering logic is unit-testable
    without a database or real files.
    """
    occurrences = summarize_incidents(incidents)
    sections: Dict[str, str] = {}
    drafted_ids: List[int] = []
    seen_signature_ids = set(occurrences) | set(SIGNATURES)
    for signature_id in seen_signature_ids:
        knowledge = SIGNATURES.get(signature_id)
        if knowledge is None:
            logger.warning(
                'No kb_signatures.SignatureKnowledge entry for '
                'detected signature %r - skipping (add one to '
                'kb_signatures.py to draft it).',
                signature_id,
            )
            continue
        sections[signature_id] = render_signature_section(
            knowledge, occurrences.get(signature_id)
        )
    for incident in incidents:
        if resolve_signature_id(incident.signature_id) in sections:
            drafted_ids.append(incident.id)

    rule_content = upsert_sections(existing_rule_content, _RULE_HEADER, sections)
    troubleshooting_content = upsert_sections(
        existing_troubleshooting_content, _TROUBLESHOOTING_HEADER, sections
    )
    return rule_content, troubleshooting_content, drafted_ids


def run(
    repo_root: Path,
    rule_path: Optional[Path] = None,
    troubleshooting_path: Optional[Path] = None,
) -> None:
    """Renders from *every* incident ever recorded (not just undrafted
    ones) so occurrence stats never regress on a later run just because
    an earlier incident already got a `kb_drafted_at` stamp - that
    column is bookkeeping for "has this row ever been surfaced in a
    draft", not a filter on what the rendered docs should reflect.
    Idempotent: re-running with no new incidents reproduces byte-
    identical output (see `test_draft_kb_is_idempotent_on_second_pass_
    with_same_incidents`).
    """
    rule_path = repo_root / (rule_path or DEFAULT_RULE_RELATIVE_PATH)
    troubleshooting_path = repo_root / (
        troubleshooting_path or DEFAULT_TROUBLESHOOTING_RELATIVE_PATH
    )

    session = SessionLocal()
    try:
        all_incidents = session.query(Incident).all()

        existing_rule = rule_path.read_text() if rule_path.is_file() else ''
        existing_troubleshooting = (
            troubleshooting_path.read_text()
            if troubleshooting_path.is_file()
            else ''
        )

        rule_content, troubleshooting_content, matched_ids = draft_kb(
            all_incidents, existing_rule, existing_troubleshooting
        )

        rule_path.parent.mkdir(parents=True, exist_ok=True)
        rule_path.write_text(rule_content)
        troubleshooting_path.parent.mkdir(parents=True, exist_ok=True)
        troubleshooting_path.write_text(troubleshooting_content)

        matched_ids_set = set(matched_ids)
        newly_drafted = [
            incident
            for incident in all_incidents
            if incident.id in matched_ids_set and incident.kb_drafted_at is None
        ]
        now = datetime.now(timezone.utc)
        for incident in newly_drafted:
            incident.kb_drafted_at = now
        session.commit()

        logger.info(
            'Drafted %s and %s from %d total incident(s) (%d newly '
            'marked kb_drafted_at). Review the diff and commit if it '
            'looks right - this job never commits/pushes itself.',
            rule_path,
            troubleshooting_path,
            len(all_incidents),
            len(newly_drafted),
        )
    finally:
        session.close()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--repo-root',
        type=Path,
        default=Path(__file__).resolve().parents[4],
        help=(
            'Root of the monorepo the .mdc/.md files live in (default: '
            'inferred from this file\'s location).'
        ),
    )
    parser.add_argument('--rule-path', type=Path, default=None)
    parser.add_argument('--troubleshooting-path', type=Path, default=None)
    args = parser.parse_args()
    run(args.repo_root, args.rule_path, args.troubleshooting_path)


if __name__ == '__main__':
    main()
