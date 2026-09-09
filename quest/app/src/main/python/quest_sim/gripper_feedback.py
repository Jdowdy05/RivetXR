"""Read opposing finger loads from one native MuJoCo solve, without mutation."""
import math


class GripperFeedback:
    def __init__(self, session):
        self.np = session.np
        self.model, self.data, self.mj = session.solver.mj_model, session.solver.mj_data, session.solver._mujoco
        self.joints = tuple(self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, name)
                            for name in ('panda_finger_joint1', 'panda_finger_joint2'))
        if min(self.joints) < 0:
            raise ValueError('Franka finger feedback joints unavailable')
        self.bodies = tuple(int(self.model.jnt_bodyid[joint]) for joint in self.joints)
        if len(set(self.bodies)) != 2:
            raise ValueError('Finger feedback requires distinct bodies')
        mapping = session.solver.mjc_body_to_newton.numpy()[0]
        self.robot_bodies = frozenset(i for i, body in enumerate(mapping) if 0 <= body < session.robot_body_count)
        self.force = self.np.empty(6)

    def read(self):
        """Return positive loads opposing closure, or None for invalid data.

        Caller must gate this on a successful recent simulation step. Geometry
        and joint axes both come from that same native solve; neither is taken
        from a newer FK pose. Self-contact and inactive proximity candidates do
        not constitute external finger load. No QDIA sampling limit applies.
        """
        result = [0., 0.]
        m, d, np = self.model, self.data, self.np
        for index in range(d.ncon):
            if d.contact.exclude[index] != 0 or d.contact.efc_address[index] < 0:
                continue
            a, b = map(int, m.geom_bodyid[d.contact.geom[index]])
            slots = [(i, -1., b) for i, body in enumerate(self.bodies) if body == a]
            slots += [(i, 1., a) for i, body in enumerate(self.bodies) if body == b]
            slots = [(i, sign) for i, sign, other in slots if other not in self.robot_bodies]
            if not slots:
                continue
            self.mj.mj_contactForce(m, d, index, self.force)
            normal_force = float(self.force[0])
            if not math.isfinite(normal_force):
                return None
            for i, sign in slots:
                projection = float(np.dot(d.contact.frame[index, :3] * sign, d.xaxis[self.joints[i]]))
                if not math.isfinite(projection):
                    return None
                result[i] += max(0., projection * max(0., normal_force))
        return tuple(result) if all(math.isfinite(v) for v in result) else None
