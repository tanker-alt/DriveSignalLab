"""Shared, map-bound defaults for controller and traffic generator."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ACTIVE_PROFILE=ROOT/'evaluation/configs/active_map_profile.json'


def read_profile(path):
    path=Path(path).resolve()
    data=json.loads(path.read_text(encoding='utf-8-sig'))
    if data.get('schema_version')!='map_profile.1':
        raise ValueError('Unsupported map profile version')
    if not data.get('opendrive_sha256') or not isinstance(data.get('junction_id'),int):
        raise ValueError('Profile lacks map fingerprint or junction ID')
    data['_path']=str(path)
    data['_files']={}
    for name,entry in data['files'].items():
        target=(path.parent/entry['path']).resolve()
        if hashlib.sha256(target.read_bytes()).hexdigest()!=entry['sha256']:
            raise ValueError('Profile file changed; regenerate profile: '+str(target))
        data['_files'][name]=str(target)
    for required in ('inventory','detection_config','approach_geometry'):
        if required not in data['_files']:
            raise ValueError('Missing profile file: '+required)
    generated=data.get('generated_map')
    if generated:
        manifest=(path.parent/generated['manifest']).resolve()
        raw=manifest.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=generated['sha256']:
            raise ValueError('Generated map manifest changed')
        design=json.loads(raw.decode('utf-8'))
        if design['opendrive_sha256']!=data['opendrive_sha256']:
            raise ValueError('Generated map/profile fingerprint mismatch')
        if data['junction_id'] not in [j['id'] for j in design['junctions']]:
            raise ValueError('Generated map/profile junction mismatch')
        data['_design']=design
    return data


def configure_arguments(parser,args,active_path=ACTIVE_PROFILE):
    """Explicit profile wins; explicit junction uses standalone mode.

    With neither, use the active profile. Never silently fall back to 1427.
    """
    selected=getattr(args,'profile',None)
    if not selected and args.junction_id is None and Path(active_path).exists():
        pointer=json.loads(Path(active_path).read_text(encoding='utf-8'))
        selected=Path(active_path).parent/pointer['profile']
    profile=read_profile(selected) if selected else None
    if profile:
        if args.junction_id is not None and args.junction_id!=profile['junction_id']:
            parser.error('--junction-id conflicts with selected profile')
        args.junction_id=profile['junction_id']
        for field in ('inventory','detection_config','stop_reference_config'):
            if hasattr(args,field):
                if getattr(args,field):
                    parser.error('Do not override '+field+' inside a profile; prepare a new profile')
                setattr(args,field,profile['_files'].get(field,''))
        print('Map profile: {} / junction {} / {}'.format(profile['map_name'],profile['junction_id'],profile['calibration_status']))
    if args.junction_id is None:
        parser.error('Choose a junction using python -m Toolkits.CollectionTools.map_setup, or pass --junction-id explicitly')
    args.host=args.host or (profile['server']['host'] if profile else 'localhost')
    args.port=args.port or (profile['server']['port'] if profile else 2000)
    return profile


def adapt_world(world,profile):
    """Use design phase slots and checked lane references on generated maps."""
    if profile and profile.get('_design'):
        from Toolkits.JunctionToolkit.project_bridge import World
        if isinstance(world,World):
            if world.design!=profile['_design']:
                raise ValueError('Conflicting generated map adapters')
            return world
        return World(world,profile['_design'])
    return world


def validate_world(world,profile):
    if profile is None:
        return
    road_map=world.get_map()
    if (road_map.name.split('/')[-1]!=profile['map_name'].split('/')[-1] or
            hashlib.sha256(road_map.to_opendrive().encode('utf-8')).hexdigest()!=profile['opendrive_sha256']):
        raise ValueError('Loaded map differs from active profile. Run python -m Toolkits.CollectionTools.map_setup again.')
    if not world.get_traffic_lights_in_junction(profile['junction_id']):
        raise ValueError('Profile junction has no traffic lights on the loaded map')
