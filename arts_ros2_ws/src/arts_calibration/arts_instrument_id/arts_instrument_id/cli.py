"""Offline analysis entry point: ``ros2 run arts_instrument_id identify``.

Deliberately offline and hardware-free. Capture happens through the launch files in
``capture/``, which write raw data; this reads that data and produces the report and the
instrument YAML. Keeping the two apart is what lets anyone re-derive a published number from
committed data without a robot on the bench.
"""

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='identify',
        description='Analyse instrument identification captures (see '
                    'docs/instrument_identification.md).',
    )
    parser.add_argument('stage', choices=['homing', 'coupling', 'geometry', 'all'],
                        help='which stage of the protocol to analyse')
    parser.add_argument('--data', type=Path, required=True,
                        help='directory of raw capture data for one instrument')
    parser.add_argument('--instrument', required=True,
                        help='instrument name, e.g. monopolar_curved_scissors')
    parser.add_argument('--part-number', required=True,
                        help='read off the instrument; 400xxx/420xxx/470xxx are different '
                             'generations and the number is required to compare anything')
    parser.add_argument('--out', type=Path, default=Path('docs/instrument_id'),
                        help='where the report is written')

    args = parser.parse_args(argv)

    if not args.data.is_dir():
        parser.error(f'no such capture directory: {args.data}')

    print(
        f'arts_instrument_id: stage={args.stage} instrument={args.instrument} '
        f'({args.part_number})\n'
        f'  data: {args.data}\n'
        f'  out:  {args.out}\n\n'
        'Analysis wiring is not implemented yet. The stage modules are, and are usable '
        'directly:\n'
        '  arts_instrument_id.homing.summarize()          Stage A statistics\n'
        '  arts_instrument_id.axis_fit.fit_circle_3d()    joint axis from a swept point\n'
        '  arts_instrument_id.coupling.fit_coupling()     Stage B matrix\n'
        '  arts_instrument_id.coupling.backlash()         Stage B hysteresis band\n'
        '  arts_instrument_id.frames.pivot_calibration()  tool frame origin\n'
        '  arts_instrument_id.geometry.observability()    before trusting any fit\n',
        file=sys.stderr,
    )
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
