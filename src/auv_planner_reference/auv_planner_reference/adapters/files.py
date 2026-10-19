from pathlib import Path

def open_evaluation_log(results_dir):
    return open(Path(results_dir) / 'evaluation_parameters.txt', 'a')
