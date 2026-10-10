import math

def json_safe(value):
    """Retain nonfinite diagnostic values without producing invalid JSON."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if hasattr(value, 'tolist'):
        return json_safe(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value
