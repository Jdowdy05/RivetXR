"""QDIA identity/contact sidecar. Read at <=10 Hz on the owning worker."""
import math
import struct
from collections import deque

DETAILS_HEADER = struct.Struct("<4sHHIIQdIIII")
DETAILS_OBJECT = struct.Struct("<III3f")
DETAILS_CONTACT = struct.Struct("<ii3f3ff")
MAX_CONTACTS = 32
CONTACTS_NOT_CURRENT = 2  # QDIA v1 bit 1; no contact counts/records accompany it.


def _normal_force_reader(solver):
    """Read net normal load only, matching pinned native mj_contactForce.

    Audited MuJoCo 10eeb8289598421cbca0b39459652ed57ec3d2e6:
    src/engine/engine_core_util.c:1076 and engine_util_misc.c:1584.
    Elliptic/frictionless contacts use the first constraint force. Pyramidal
    normals sum the 2*(dim-1) edge forces in native order. Both then subtract
    contact adhesion. The returned reader belongs to this one completed solve;
    never retain it across a step/rebuild. Published forces still use MuJoCo's
    public API. Older duck-typed diagnostic fixtures without native arrays
    retain activity-only ordering; the actual Session has all required arrays.
    """
    model, data, mj = solver.mj_model, solver.mj_data, solver._mujoco
    contact = data.contact
    required = ((contact, 'efc_address'), (contact, 'dim'), (contact, 'adhesion'),
                (data, 'efc_force'), (model, 'opt'), (mj, 'mjtCone'))
    if any(not hasattr(owner, name) for owner, name in required):
        return None
    cone = int(model.opt.cone)
    if cone not in (int(mj.mjtCone.mjCONE_PYRAMIDAL), int(mj.mjtCone.mjCONE_ELLIPTIC)):
        raise ValueError('Unsupported MuJoCo contact cone')
    pyramidal = cone == int(mj.mjtCone.mjCONE_PYRAMIDAL)
    addresses, dimensions, adhesion, forces = contact.efc_address, contact.dim, contact.adhesion, data.efc_force

    def read(index):
        address = int(addresses[index])
        if address < 0:
            return 0.
        dimension = int(dimensions[index])
        if dimension not in (1, 3, 4, 6):
            raise ValueError('Unsupported MuJoCo contact dimension')
        count = 2*(dimension-1) if pyramidal and dimension > 1 else 1
        if address+count > len(forces):
            raise ValueError('MuJoCo contact force address exceeds active storage')
        normal = float(forces[address])
        if count > 1:
            normal = 0.
            # Explicit sequential addition preserves native summation order;
            # Python 3.12 sum()/NumPy may use compensated/pairwise reductions.
            for offset in range(count):
                normal += float(forces[address+offset])
        normal -= float(adhesion[index])
        if not math.isfinite(normal):
            raise FloatingPointError('object contact normal load is nonfinite')
        return normal
    return read


def _contact_candidates(solver, object_bodies):
    """Select a fair diagnostic subset without per-candidate native API calls.

    Count every native contact involving an object as before. For truncated
    packets, positive net normal loads take priority over active zero/negative
    loads, then inactive proximity contacts. Represent body pairs round-robin,
    with their initial order interleaved across object identities so a busy
    first object cannot consume every representative. Shared object/object
    pairs are visited once. Keep at most 32 candidates per pair and priority
    class: no more could fit in the final packet.
    """
    data = solver.mj_data
    contact = data.contact
    native_bodies = solver.mjc_body_to_newton.numpy()[0]
    geom_bodies = solver.mj_model.geom_bodyid
    exclude = getattr(contact, 'exclude', None)
    addresses = getattr(contact, 'efc_address', None)
    normal_load = _normal_force_reader(solver)
    owners = {body: deque() for body in object_bodies}
    groups, total = {}, 0
    for index in range(int(data.ncon)):
        geom_a, geom_b = contact.geom[index]
        body_a, body_b = int(native_bodies[geom_bodies[geom_a]]), int(native_bodies[geom_bodies[geom_b]])
        if body_a not in owners and body_b not in owners:
            continue
        total += 1
        pair = (min(body_a, body_b), max(body_a, body_b))
        if pair not in groups:
            groups[pair] = ([], [], [])
            for body in dict.fromkeys(pair):
                if body in owners:
                    owners[body].append(pair)
        active = ((exclude is None or exclude[index] == 0)
                  and (addresses is None or addresses[index] >= 0))
        loaded = active and normal_load is not None and normal_load(index) > 0.
        priority = 0 if loaded else 1 if active else 2
        bucket = groups[pair][priority]
        if len(bucket) < MAX_CONTACTS:
            bucket.append((index, body_a, body_b))

    if total <= MAX_CONTACTS:
        # Complete packets retain their original native order byte-for-byte.
        return total, sorted((row for buckets in groups.values() for bucket in buckets for row in bucket),
                             key=lambda row: row[0])

    pair_order, seen = [], set()
    while len(pair_order) < len(groups):
        for queue in owners.values():
            while queue and queue[0] in seen:
                queue.popleft()
            if queue:
                pair = queue.popleft()
                seen.add(pair)
                pair_order.append(pair)

    selected = []
    for activity in (0, 1, 2):
        for depth in range(MAX_CONTACTS):
            advanced = False
            for pair in pair_order:
                bucket = groups[pair][activity]
                if depth < len(bucket):
                    selected.append(bucket[depth])
                    advanced = True
                    if len(selected) == MAX_CONTACTS:
                        return total, selected
            if not advanced:
                break
    return total, selected


def details_bytes(session, include_contacts=True):
    if type(include_contacts) is not bool:
        raise ValueError("include_contacts must be boolean")
    objects = []
    if session.box_body is not None:
        objects.append((0, session.box_body, 1, .05, .05, .05))
    objects.extend((obj.id, session.object_bodies[obj.id], 1, *obj.half_extents) for obj in session.objects)
    records, total = [], 0
    contacts_current = getattr(session, "_contacts_current", False)
    # A metadata-only response does not sample contacts; the caller knows the
    # requested mode and must not render these zero counts as a no-contact claim.
    if include_contacts and contacts_current and objects:
        solver, np = session.solver, session.np
        data = solver.mj_data
        total, candidates = _contact_candidates(solver, [record[1] for record in objects])
        forces = np.empty(6, dtype=np.float64)
        for index, body_a, body_b in candidates:
            solver._mujoco.mj_contactForce(solver.mj_model, data, index, forces)
            values = (*data.contact.pos[index], *data.contact.frame[index][:3], float(forces[0]))
            if not all(math.isfinite(v) and abs(v) <= np.finfo(np.float32).max for v in values):
                raise FloatingPointError("object contact diagnostics contain nonfinite/out-of-range values")
            records.append(DETAILS_CONTACT.pack(body_a, body_b, *values))
    return (DETAILS_HEADER.pack(b"QDIA", 1, DETAILS_HEADER.size, session.generation, session.model.body_count,
                                session.step_index, session.sim_time, len(objects), len(records), total,
                                int(total > len(records)) | (CONTACTS_NOT_CURRENT if include_contacts and not contacts_current else 0))
            + b"".join(DETAILS_OBJECT.pack(*record) for record in objects) + b"".join(records))
