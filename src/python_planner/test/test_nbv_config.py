"""Keep split NBV tuning aligned with the parameters declared by the planner."""

import ast
from pathlib import Path

import yaml


def test_split_nbv_config_contains_supported_tuning_parameters():
    source_root = Path(__file__).parents[2]
    tree = ast.parse((source_root / 'python_planner/python_planner/get_NBV.py').read_text())
    defaults = next(ast.literal_eval(node.value) for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'DEFAULT_NBV_PARAMETERS'
                            for target in node.targets))
    config = source_root / 'bringup/config'
    tuning = yaml.safe_load((config / 'nbv_params.yaml').read_text())['planner_node']['ros__parameters']
    main = yaml.safe_load((config / 'params.yaml').read_text())['planner_node']['ros__parameters']
    assert set(tuning) == {f'nbv_{key}' for key in defaults}
    assert not set(tuning).intersection(main)
    assert tuning['nbv_complexity_weight'] == 15.
