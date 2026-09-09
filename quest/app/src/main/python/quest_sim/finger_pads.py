"""Five massless contact boxes on each pinned Panda finger's inward flat face.

Template measured from finger.stl SHA256
eade0765d87446c8833d88511a4c46573baba2e8eacc15b975aecd123d740296.
The mesh-local inward face is y=0.00007057935 m. Boxes face y=0,
only 70.6 micrometres proud, with 0.4 mm gaps along z. Their 16 x 1 x
2.4 mm volumes stay inside the finger's side/tip envelope. The mesh remains
underneath and can share loaded contacts when penetration exceeds the outset;
five pad shapes do not imply five contacts or pad-only contact generation.

No material tuning, assets, bodies, joints or inertia are changed here. The
caller may configure materials using the ten returned shape indices.
"""

FINGERS = ('panda_leftfinger', 'panda_rightfinger')
PAD_CENTRES_Z = (.0375, .0403, .0431, .0459, .0487)
PAD_HALF_EXTENTS = (.008, .0005, .0012)


def add_finger_pads(builder):
    """Add ten collidable BOX shapes; return left-five then right-five indices.

    Call once after importing the unscaled Panda URDF and before finalize().
    Both namespaced and normalized finger body names are accepted. The actual
    imported mesh-to-body transform supplies right-finger mirroring, including
    its URDF 180-degree Z rotation; no world/body-side sign is guessed.
    """
    import newton
    import warp as wp

    labels = tuple(f'{finger}_pad_{i}' for finger in FINGERS for i in range(5))
    if any(label in builder.shape_label for label in labels):
        raise ValueError('Panda finger pads have already been added')
    targets = []
    for finger in FINGERS:
        bodies = [i for i, label in enumerate(builder.body_label) if label.rsplit('/', 1)[-1] == finger]
        if len(bodies) != 1:
            raise ValueError(f'Expected one existing {finger} body')
        body = bodies[0]
        meshes = [i for i, owner in enumerate(builder.shape_body) if owner == body
                  and builder.shape_type[i] in (newton.GeoType.MESH, newton.GeoType.CONVEX_MESH)
                  and builder.shape_flags[i] & newton.ShapeFlags.COLLIDE_SHAPES]
        if len(meshes) != 1:
            raise ValueError(f'Expected one original mesh collider on {finger}')
        mesh = meshes[0]
        if tuple(builder.shape_scale[mesh]) != (1., 1., 1.) or builder.shape_collision_group[mesh] == 0:
            raise ValueError('Panda pads require unscaled, collision-enabled finger meshes')
        targets.append((body, mesh))

    # Complete preflight for both sides before changing the builder. Density
    # zero skips _update_body_mass in the pinned ordinary add_shape_box path.
    added = []
    for side, (body, mesh) in enumerate(targets):
        cfg = builder.default_shape_cfg.copy()
        cfg.density = 0.
        cfg.is_site = False
        cfg.has_shape_collision = True
        cfg.is_visible = True
        cfg.collision_group = builder.shape_collision_group[mesh]
        body_from_mesh = builder.shape_transform[mesh]
        for index, z in enumerate(PAD_CENTRES_Z):
            pose = wp.transform_multiply(body_from_mesh, wp.transform((0., .0005, z), wp.quat_identity()))
            added.append(builder.add_shape_box(body, xform=pose, hx=PAD_HALF_EXTENTS[0],
                hy=PAD_HALF_EXTENTS[1], hz=PAD_HALF_EXTENTS[2], cfg=cfg, label=labels[side*5+index]))
    return tuple(added)
