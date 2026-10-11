"""Translate the configured camera extrinsic to static publisher CLI flags."""
import yaml


def camera_transform_arguments(default_params_file, params_file):
    mount = {}
    for path in (default_params_file, params_file):
        with open(path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        mount.update(config.get('camera_static_transform', {}).get('ros__parameters', {}))
    fields = (
        ('--frame-id', 'frame_id'), ('--child-frame-id', 'child_frame_id'),
        ('--x', 'translation.x'), ('--y', 'translation.y'), ('--z', 'translation.z'),
        ('--qx', 'rotation.x'), ('--qy', 'rotation.y'),
        ('--qz', 'rotation.z'), ('--qw', 'rotation.w'),
    )
    return [value for flag, key in fields for value in (flag, str(mount[key]))]
