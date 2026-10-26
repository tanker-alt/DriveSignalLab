"""Connect to the bundled Cross3 map without loading, ticking or spawning."""
import json
from pathlib import Path
from .map_profile import ROOT, read_profile, adapt_world, validate_world


def connect(host='localhost', port=2000):
    """Return (client, adapted_world, profile); caller owns simulation/actions."""
    import carla
    profile = read_profile(ROOT / 'map_profiles/generated_cross3/profile.json')
    client = carla.Client(host, port)
    client.set_timeout(15)
    world = adapt_world(client.get_world(), profile)
    validate_world(world, profile)
    return client, world, profile


def detection_config(profile):
    return json.loads(Path(profile['_files']['detection_config']).read_text(encoding='utf-8'))
