#!/usr/bin/env python3
"""Copy real session PNGs and metadata into a traceable combined dataset.

First session filenames are preserved so its sorted-index holdout stays fixed.
Later sessions are prefixed zz_ and sort after the original dataset.
"""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sessions',type=Path,nargs='+',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists():
        raise RuntimeError('Combined directory already exists; use a new output path')
    loaded=[]; all_hashes=set()
    for index,session in enumerate(args.sessions):
        session=session.resolve()
        with (session/'capture_metadata.csv').open() as f:
            reader=csv.DictReader(f);fields=reader.fieldnames
            rows={Path(r['saved_path']).name:r for r in reader if r['accepted'].lower()=='true'}
        paths=sorted((session/'images').glob('*.png'))
        if not paths:
            raise RuntimeError(f'No real images: {session}')
        for path in paths:
            if path.name not in rows:
                raise RuntimeError(f'Image missing accepted capture metadata: {path}')
            sha=hashlib.sha256(path.read_bytes()).hexdigest()
            if sha in all_hashes:
                raise RuntimeError(f'Exact duplicate image: {path}; review before merging')
            all_hashes.add(sha)
            name=path.name if index==0 else f'zz_{index:02d}_{session.name}__{path.name}'
            loaded.append((path,name,rows[path.name],sha))
    (args.output/'images').mkdir(parents=True)
    provenance=[]
    with (args.output/'capture_metadata.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for path,name,row,sha in loaded:
            dest=args.output/'images'/name
            shutil.copy2(path,dest)
            assert hashlib.sha256(dest.read_bytes()).hexdigest()==sha
            row=dict(row);row['saved_path']=str(dest.resolve());writer.writerow(row)
            provenance.append(dict(combined_file=name,original_file=str(path),sha256=sha))
    (args.output/'merge_manifest.json').write_text(json.dumps(dict(
        sessions=[str(s.resolve()) for s in args.sessions],images=provenance,
        note='Unmodified byte copies. Original first-session filenames and their sorted holdout indices preserved.'),indent=2))
    print(f'Merged {len(loaded)} real images into {args.output}')

if __name__=='__main__':
    main()
