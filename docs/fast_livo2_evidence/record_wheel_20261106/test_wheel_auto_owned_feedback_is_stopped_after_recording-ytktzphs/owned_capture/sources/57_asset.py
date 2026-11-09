"""Read the immutable navigation projection and bind it to its 3D source."""
import hashlib
import json
from pathlib import Path
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_map(folder):
    folder=Path(folder).resolve()
    manifest=json.loads((folder/'manifest.json').read_text())
    if json.loads((folder/'COMMITTED').read_text())['manifest_sha256'] != sha(folder/'manifest.json'):
        raise ValueError('navigation manifest checksum mismatch')
    if manifest.get('schema_version') != 1 or manifest.get('kind') != 'fast_livo_navigation_map_candidate' or (folder/'FAILED.json').exists():
        raise ValueError('unsupported or failed navigation map')
    for name,key in [('map.pgm','map_image_sha256'),('map.yaml','map_yaml_sha256')]:
        if sha(folder/name) != manifest[key]: raise ValueError('navigation map checksum mismatch: '+name)
    source=Path(manifest['source_archive'])
    if sha(source/'manifest.json') != manifest['source_manifest_sha256']:
        raise ValueError('source archive manifest changed')
    original=json.loads((source/'manifest.json').read_text())
    if (original['map_uuid'],original['graph_version']) != (manifest['map_uuid'],manifest['graph_version']):
        raise ValueError('navigation map source identity mismatch')
    config=yaml.safe_load((folder/'map.yaml').read_text())
    if (config['image'] != 'map.pgm' or config['resolution'] != manifest['resolution'] or config['origin'] != manifest['origin']
            or manifest['frame'] != 'map'):
        raise ValueError('navigation map metadata mismatch')
    return manifest
