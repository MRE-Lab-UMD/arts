"""Report and instrument-YAML emission.

Same contract as the benchmark harness in AGENTS.md decision 10: a markdown report for humans
and a JSON sidecar for machines, committed next to the raw capture data so every published
number is auditable rather than merely asserted.

The instrument YAML this writes is the artifact ``arts_description`` and ``arts_dv_tool_driver``
actually consume. Every value in it carries an uncertainty and the run that produced it —
that provenance is the whole point, and it is what distinguishes these numbers from the
transcribed ones they replace.
"""

from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _plain(obj: Any) -> Any:
    """Recursively convert numpy and dataclass values into YAML/JSON-safe Python."""
    if is_dataclass(obj) and not isinstance(obj, type):
        return _plain(asdict(obj))
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _plain(obj.tolist())
    if isinstance(obj, (np.floating, float)):
        value = float(obj)
        return None if np.isnan(value) else value
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    if isinstance(obj, (np.bool_, bool)):
        return bool(obj)
    return obj


def write_instrument_yaml(
    path: str | Path,
    instrument: str,
    part_number: str,
    results: dict[str, Any],
    *,
    notes: str = '',
) -> Path:
    """Emit the instrument definition consumed by arts_description.

    ``part_number`` is not optional and not cosmetic. dVRK's published values are mostly for
    400xxx (S/Si) and 420xxx parts while Xi variants are 470xxx; an unlabelled parameter set
    cannot be compared against anything, and a disagreement traced to a generation difference
    is a result rather than an error.
    """
    if not part_number or part_number.upper() in {'TBD', 'UNKNOWN'}:
        raise ValueError(
            'part_number is required — read it off the instrument before measuring it. '
            'See docs/instrument_identification.md, Stage C.'
        )

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    document = {
        'instrument': instrument,
        'part_number': part_number,
        'generated': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'units': {'length': 'm', 'angle': 'rad'},
        'provenance': (
            'Measured from a physically owned instrument per '
            'docs/instrument_identification.md. Not transcribed from any published source.'
        ),
        'notes': notes,
        **_plain(results),
    }
    path.write_text(yaml.safe_dump(document, sort_keys=False, default_flow_style=False))
    return path


def write_report(
    directory: str | Path,
    instrument: str,
    part_number: str,
    sections: dict[str, str],
    machine_readable: dict[str, Any],
) -> tuple[Path, Path]:
    """Write ``<instrument>_<date>.md`` and ``.json`` into ``directory``.

    ``sections`` maps a heading to already-formatted markdown; each stage module is
    responsible for describing its own output, since only it knows which caveats matter.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stem = f'{instrument}_{datetime.now().strftime("%Y%m%d")}'

    body = [
        f'# Instrument identification — {instrument} ({part_number})',
        '',
        f'Generated {datetime.now(timezone.utc).isoformat(timespec="seconds")}.',
        '',
        'Protocol: `docs/instrument_identification.md`. Raw capture data is committed '
        'alongside this report.',
        '',
    ]
    for heading, text in sections.items():
        body += [f'## {heading}', '', text, '']

    md_path = directory / f'{stem}.md'
    json_path = directory / f'{stem}.json'
    md_path.write_text('\n'.join(body))
    json_path.write_text(
        json.dumps(
            {'instrument': instrument, 'part_number': part_number, **_plain(machine_readable)},
            indent=2,
        )
    )
    return md_path, json_path
