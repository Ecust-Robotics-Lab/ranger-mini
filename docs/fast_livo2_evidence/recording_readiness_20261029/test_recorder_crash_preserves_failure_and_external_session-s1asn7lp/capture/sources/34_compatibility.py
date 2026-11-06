#!/usr/bin/env python3
"""Bind incomplete historical map provenance to explicit offline-only evidence."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for data in iter(lambda: handle.read(1024*1024), b''):
            result.update(data)
    return result.hexdigest()


def declaration(archive, evidence):
    archive = Path(archive).resolve()
    digest = sha(archive/'manifest.json')
    if json.loads((archive/'COMMITTED').read_text())['manifest_sha256'] != digest:
        raise ValueError('source manifest does not match COMMITTED')
    return {'schema_version': 1, 'mode': 'offline_experiment', 'source_manifest_sha256': digest,
            'physical_sync_validated': False, 'absolute_accuracy_validated': False,
            'limitations': ['Historical archive lacks complete attested sensor/calibration/source identity.',
                           'Referenced evidence is traceability, not physical calibration validation.',
                           'Same-capture localization is a function test; no independent-session or accuracy claim.'],
            'evidence_files_sha256': {str(Path(p).resolve()): sha(p) for p in evidence}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive', type=Path, required=True)
    p.add_argument('--evidence', type=Path, action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    data = declaration(args.archive, args.evidence)
    if args.execute:
        with args.output.open('x') as handle:
            handle.write(json.dumps(data, indent=2)+'\n')
    else:
        print(json.dumps({'action': 'DRY-RUN', 'output': str(args.output), 'declaration': data}, indent=2))


if __name__ == '__main__':
    main()
