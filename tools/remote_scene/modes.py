"""One producer connection, with an explicit reconstruction location."""
from .fusion import fuse_batch

GEOMETRY_MODES=('prepared','unprepared','points')


def build_batch(frames,cameras,*,geometry,rgb=None,observation_map=None,tracking_valid=True,
                retention_enabled=False,retention_ms=30000,**options):
    if geometry not in GEOMETRY_MODES:raise ValueError('geometry must be prepared, unprepared or points')
    if rgb is not None and type(rgb) is not bool:raise ValueError('RGB mode must be boolean')
    if not isinstance(cameras,(list,tuple)):cameras=list(cameras)
    required_rgb=any(c.camera_id>4 or c.image_format!='Y8' for c in cameras)
    use_rgb=required_rgb or bool(rgb) or retention_enabled or observation_map is not None
    if geometry=='points':
        if use_rgb:raise ValueError('RGB/six-camera/retained scenes require prepared or unprepared geometry')
        return fuse_batch(frames if tracking_valid else [],cameras,**options)
    options.pop('voxel_size_m',None);options.pop('max_points',None)
    if use_rgb:
        from .rgb_surface import build_rgb_batch
        return build_rgb_batch(frames,cameras,geometry=geometry,observation_map=observation_map,tracking_valid=tracking_valid,
                              retention_enabled=retention_enabled,retention_ms=retention_ms,**options)
    from .surface import build_surface_batch
    return build_surface_batch(frames if tracking_valid else [],cameras,geometry=geometry,**options)
