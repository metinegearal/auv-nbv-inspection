"""Capture configuration and content hashes without modifying the reference."""
import hashlib
import json
import pathlib
import sys
import types

package = pathlib.Path(__file__).resolve().parents[1]
root = package.parent
sys.path[:0] = [str(root / 'auv_planner_reference'), str(root / 'auv_contracts')]
from auv_planner_reference.domain.configuration import load_configuration
from auv_planner_reference.domain.nbv_parameters import DEFAULT_NBV_PARAMETERS

parameters = {}
def read(name, default):
    parameters[name] = default
    return default
load_configuration(read)
(package / 'migration_evidence/reference_defaults.json').write_text(json.dumps(parameters, indent=2) + '\n')
cpp = '#include "auv_inspection_planner/migration/core.hpp"\nnamespace inspection::migration {\n'
cpp += 'Json defaults() { return Json::parse(R"json(' + json.dumps(parameters) + ')json"); }\n'
cpp += 'Json nbv_defaults() { return Json::parse(R"json(' + json.dumps(DEFAULT_NBV_PARAMETERS) + ')json"); }\n}\n'
(package / 'src/migration/configuration.cpp').write_text(cpp)
manifest = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((root / 'auv_planner_reference').rglob('*')) if p.is_file()
            and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts}
(package / 'migration_evidence/reference_sha256.json').write_text(json.dumps(manifest, indent=2) + '\n')
