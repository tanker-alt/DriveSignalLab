"""Load the vendored collector under a private namespace, without a second loop."""
import importlib
import importlib.util
from pathlib import Path
import sys


def create(world, junction_id, source_root, output_root):
    package = Path(source_root) / 'evaluation'
    name = '_drivesignal_handoff_evaluation'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            name, str(package/'__init__.py'), submodule_search_locations=[str(package)])
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    api = importlib.import_module(name+'.api')
    return api.create_collector(world, api.RoadSpec(junction_id=junction_id),
                                output_root=output_root, mode='mdrive_owned_tick')
