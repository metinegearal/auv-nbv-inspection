"""Composition facade retaining the standalone evaluator API and defaults."""
if not __package__:
    # Retain direct file execution as well as normal package/module execution.
    import importlib.util
    from pathlib import Path
    import sys
    package_dir = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location(
        'benchmark_evaluator', package_dir / '__init__.py',
        submodule_search_locations=[str(package_dir)])
    package = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)
    __package__ = spec.name
from .domain.evaluation import MapEvaluationConfig

DEFAULT_CONFIG = MapEvaluationConfig(
    '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/results/final_map_ros2.bt',
    '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Tools/3D_scan/GT/objects/SM_MERGED_pipe_full.OBJ')
    # '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/Tools/3D_scan/GT/objects/SM_MERGED_pipe_m_l100_390.OBJ')

def evaluate_map_accuracy(headless=False, config=DEFAULT_CONFIG):
    from .adapters.map_geometry import Open3DMapEvaluation
    from .use_cases.map_evaluation import EvaluateMap
    return EvaluateMap(Open3DMapEvaluation(config), print).execute(headless)

if __name__ == '__main__':
    evaluate_map_accuracy()
