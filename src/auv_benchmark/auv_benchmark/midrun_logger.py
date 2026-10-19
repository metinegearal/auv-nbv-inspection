"""Compatibility exports; storage and visualization have separate owners."""
from auv_telemetry.file_sinks import MidrunLogger
from .plot_adapter import generate_results_plot
from datetime import datetime

def default_run_id():
    return datetime.now().strftime('%Y%m%d_%H%M%S')
