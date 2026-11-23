"""Explicit source migration for the reviewed pending-VIO scheduling fix."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_active_frontend():
    lock=json.loads((ROOT/'config/fast_livo2/active_frontend.lock.json').read_text())
    if lock.get('schema_version')!=1 or lock.get('change_kind')!='finish_pending_vio_without_extra_scan':
        raise ValueError('unsupported active frontend source lock')
    for name,digest in lock['files_sha256'].items():
        if sha(ROOT/name)!=digest:raise ValueError('active frontend source/binary changed: '+name)
    source=ROOT/lock['source_directory'];manifest=json.loads((source/'latency_source_manifest.json').read_text())
    parent=json.loads((ROOT/'ws_fast_livo/online_frontend_source/online_source_manifest.json').read_text())
    if manifest['variant']!='candidate' or manifest['parent_manifest']!=parent:
        raise ValueError('active frontend parent/source mismatch')
    for name,digest in manifest['files_sha256'].items():
        if sha(source/name)!=digest:raise ValueError('active frontend source changed: '+name)
    changed={name for name,digest in manifest['files_sha256'].items() if parent['files_sha256'].get(name)!=digest}
    if changed!={'src/LIVMapper.cpp','src/ranger_livo_replay.cpp'}:
        raise ValueError('frontend upgrade exceeds reviewed scheduling/replay changes')
    return lock

def declaration(archive,current,legacy,persist=True):
    """Return a bound sidecar; never edit the immutable map or fake its source."""
    lock=verify_active_frontend();archive=Path(archive);manifest=json.loads((archive/'manifest.json').read_text())
    if json.loads((archive/'COMMITTED').read_text())['manifest_sha256']!=sha(archive/'manifest.json'):
        raise ValueError('map manifest changed')
    definitions=current['definitions']['frontend_sources_sha256']
    source_digest=hashlib.sha256(json.dumps(definitions,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    if (source_digest!=current['frontend_sources_sha256'] or
            definitions['binary_sha256']!=lock['files_sha256'][lock['binary']] or
            definitions['latency_patch_sha256']!=lock['files_sha256']['patches/fast_livo2_finish_pending_vio.patch']):
        raise ValueError('current frontend identity differs from reviewed active binary')
    original=manifest['provenance']
    keys=('sensor_configuration_sha256','time_contract_sha256','frontend_sources_sha256')
    if all(original.get(k)==current[k] for k in keys):return None
    if (any(original.get(k)!=current[k] for k in keys[:2]) or
            any(original.get(k)!=legacy[k] for k in keys)):
        raise ValueError('map is not from the exact reviewed frontend predecessor, sensor and time contract')
    evidence={str((ROOT/name).resolve()):digest for name,digest in lock['files_sha256'].items()}
    evidence[str((ROOT/'config/fast_livo2/active_frontend.lock.json').resolve())]=sha(ROOT/'config/fast_livo2/active_frontend.lock.json')
    data=dict(schema_version=1,mode='frontend_timing_upgrade',change_kind=lock['change_kind'],
        source_manifest_sha256=sha(archive/'manifest.json'),map_uuid=manifest['map_uuid'],
        sensor_configuration_sha256=current[keys[0]],time_contract_sha256=current[keys[1]],
        from_frontend_sources_sha256=legacy[keys[2]],to_frontend_sources_sha256=current[keys[2]],
        physical_sync_validated=False,absolute_accuracy_validated=False,
        limitations=['User authorized promotion of reviewed pending-VIO scheduling fix; retains three-sensor VIO and original +100ms contract.',
            'Recorded long-run differences and baseline repeatability remain in evidence; no accuracy or live latency claim.',
            'Independent C2/C1 geometric validation is still required for every session.'],evidence_files_sha256=evidence)
    path=ROOT/'.runtime/frontend_upgrades'/data['source_manifest_sha256']/(current[keys[2]]+'.json')
    content=json.dumps(data,indent=2,sort_keys=True)+'\n'
    if path.exists() and path.read_text()!=content:raise ValueError('preserve changed frontend upgrade declaration: '+str(path))
    if persist and not path.exists():
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(content)
    return path
