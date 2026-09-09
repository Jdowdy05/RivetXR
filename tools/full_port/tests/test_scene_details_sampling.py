"""QDIA must represent both fingers/objects when native contacts exceed capacity."""
from collections import Counter
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'quest/app/src/main/python'))
from quest_sim.scene_details import details_bytes, DETAILS_HEADER, DETAILS_CONTACT
from quest_sim.scene_objects import SceneObject


def fixture(pairs, object_bodies=(12,), active=None, normal=None):
    import numpy as np
    # Native geom and body indices intentionally differ from Newton body IDs.
    bodies = sorted({-1, *object_bodies, *(body for pair in pairs for body in pair)})
    native = {body:index for index,body in enumerate(bodies)}
    pairs_native = np.array([[native[a],native[b]] for a,b in pairs], dtype=np.int32)
    count=len(pairs)
    active = [True]*count if active is None else active
    normal = [(index+1)/100 if active[index] else 0 for index in range(count)] if normal is None else normal
    contact = SimpleNamespace(geom=pairs_native, pos=np.zeros((count,3)),
        frame=np.tile([0,0,1,1,0,0,0,1,0],(count,1)), exclude=np.zeros(count,dtype=np.int32),
        efc_address=np.array([i*3 if value else -1 for i,value in enumerate(active)]),
        dim=np.full(count,3),adhesion=np.zeros(count))
    calls=[]
    def force(_model,_data,index,value):
        calls.append(index); value[:]=0; value[0]=normal[index]
    solver=SimpleNamespace(mj_model=SimpleNamespace(geom_bodyid=np.arange(len(bodies)),opt=SimpleNamespace(cone=1)),
        mj_data=SimpleNamespace(ncon=count,contact=contact,efc_force=np.array([[f,0,0] for f in normal]).reshape(-1)),
        mjc_body_to_newton=SimpleNamespace(numpy=lambda:np.array([bodies])),
        _mujoco=SimpleNamespace(mj_contactForce=force,mjtCone=SimpleNamespace(mjCONE_PYRAMIDAL=0,mjCONE_ELLIPTIC=1)))
    objects=tuple(SceneObject(index+1,(0,0,1,0,0,0,1),(.025,)*3) for index in range(len(object_bodies)))
    session=SimpleNamespace(np=np,solver=solver,objects=objects,box_body=None,
        object_bodies={obj.id:body for obj,body in zip(objects,object_bodies)},
        model=SimpleNamespace(body_count=max(bodies)+1),generation=2,step_index=9,sim_time=.045,
        _contacts_current=True)
    return session,calls


def unpack(session,sample=True):
    blob=details_bytes(session,sample)
    header=DETAILS_HEADER.unpack_from(blob)
    contacts=[DETAILS_CONTACT.unpack_from(blob,48+24*header[7]+36*i) for i in range(header[8])]
    return header,contacts


class SceneDetailsSamplingTests(unittest.TestCase):
    def test_active_zero_prefix_cannot_hide_later_loaded_finger_contacts(self):
        # Geometry-only pads: every candidate is active, but first16 left
        # contacts carry zero load. Fairness alone selected no loaded left row.
        pairs=[(10,12)]*21+[(11,12)]*25
        session,calls=fixture(pairs,normal=[0]*16+[1]*5+[0]*11+[1]*14)
        header,records=unpack(session)
        self.assertEqual(header[8:],(32,46,1))
        loaded=Counter(tuple(sorted(record[:2])) for record in records if record[-1]>0)
        self.assertEqual(loaded,{(10,12):5,(11,12):14})
        self.assertEqual(len(calls),32)

    def test_native_prefix_cannot_starve_second_loaded_finger(self):
        # Reproduce 58 native candidates: 16 inactive,21 left,21 right.
        session,calls=fixture([(-1,12)]*16+[(10,12)]*21+[(11,12)]*21,
                              active=[False]*16+[True]*42)
        header,records=unpack(session)
        self.assertEqual(header[7:],(1,32,58,1))
        counts=Counter(tuple(sorted(record[:2])) for record in records)
        self.assertEqual(counts,{(10,12):16,(11,12):16})
        self.assertTrue(all(record[-1]>0 for record in records))
        self.assertEqual(len(calls),32)
        self.assertEqual(len(set(calls)),32)

    def test_more_than_32_pairs_still_represent_later_object(self):
        pairs=[(12,body) for body in range(20,60)]+[(13,-1)]*40
        session,calls=fixture(pairs,object_bodies=(12,13))
        header,records=unpack(session)
        self.assertEqual(header[7:],(2,32,80,1))
        self.assertTrue(any(13 in record[:2] for record in records))
        # One representative per pair precedes repeated contacts from that pair.
        self.assertEqual(len({tuple(sorted(record[:2])) for record in records}),32)
        self.assertEqual(len(calls),32)

    def test_shared_object_pair_is_deduplicated_and_small_packets_keep_every_contact(self):
        pairs=[(12,13),(13,12),(12,-1),(13,10),(12,11)]
        session,calls=fixture(pairs,object_bodies=(12,13))
        header,records=unpack(session)
        self.assertEqual(header[7:],(2,5,5,0))
        self.assertCountEqual(calls,range(5))
        self.assertEqual(len(records),5)
        self.assertEqual([record[:2] for record in records],pairs)
        self.assertEqual(Counter(tuple(sorted(record[:2])) for record in records),
                         Counter(tuple(sorted(pair)) for pair in pairs))

    def test_truncated_shared_pairs_are_not_sampled_twice_through_two_owners(self):
        session,calls=fixture([(12,13)]*24+[(13,12)]*24+[(12,10)]*24+[(13,11)]*24,
                              object_bodies=(12,13))
        header,records=unpack(session)
        self.assertEqual(header[8:],(32,96,1))
        self.assertEqual(len(set(calls)),32)
        counts=Counter(tuple(sorted(record[:2])) for record in records)
        self.assertEqual(set(counts),{(12,13),(10,12),(11,13)})
        self.assertLessEqual(max(counts.values())-min(counts.values()),1)

    def test_priority_within_pair_retains_total_and_later_active_records(self):
        session,calls=fixture([(10,12)]*80,active=[False]*40+[True]*40)
        header,records=unpack(session)
        self.assertEqual(header[8:],(32,80,1))
        self.assertTrue(all(index>=40 for index in calls))
        self.assertTrue(all(record[-1]>0 for record in records))

    def test_stale_and_mapping_only_never_read_contact_storage(self):
        session,calls=fixture([(10,12)]*60)
        session.solver.mj_data.contact=None
        session._contacts_current=False
        self.assertEqual(unpack(session)[0][8:],(0,0,2))
        self.assertEqual(unpack(session,False)[0][8:],(0,0,0))
        session._contacts_current=True
        self.assertEqual(unpack(session,False)[0][8:],(0,0,0))
        self.assertEqual(calls,[])

    def test_selected_nonfinite_values_fail_and_signed_forces_are_not_clipped(self):
        session,calls=fixture([(10,12),(11,12)])
        def force(_model,_data,index,value):
            calls.append(index); value[:]=0; value[0]=-.25
        session.solver._mujoco.mj_contactForce=force
        _,records=unpack(session)
        self.assertTrue(all(record[-1]==-.25 for record in records))
        session.solver.mj_data.contact.pos[1,0]=math.nan
        with self.assertRaises(FloatingPointError):unpack(session)


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'native MuJoCo required')
class NativeNormalForceTests(unittest.TestCase):
    def test_decoding_matches_native_both_cones_all_dimensions_and_adhesion(self):
        import mujoco
        import numpy as np
        from quest_sim.scene_details import _normal_force_reader
        for cone in ('pyramidal','elliptic'):
            for dim in (1,3,4,6):
                with self.subTest(cone=cone,dim=dim):
                    xml=f'''<mujoco><option timestep="0.005" cone="{cone}"/>
                      <default><geom condim="{dim}" friction="1 .005 .0001" adhesion=".25"/></default>
                      <worldbody><geom type="plane" size="1 1 .1"/>
                        <body name="cube" pos="0 0 .03"><freejoint/>
                          <geom type="box" size=".05 .05 .025" mass=".125"/>
                        </body></worldbody>
                    </mujoco>'''
                    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
                    data.qvel[0]=.07;data.qvel[5]=.6
                    solver=SimpleNamespace(mj_model=model,mj_data=data,_mujoco=mujoco)
                    comparisons=0;loaded=0;adhesive=0
                    for _ in range(80):
                        mujoco.mj_step(model,data)
                        read=_normal_force_reader(solver)
                        self.assertIsNotNone(read)
                        for index in range(data.ncon):
                            value=np.empty(6);mujoco.mj_contactForce(model,data,index,value)
                            self.assertEqual(read(index),float(value[0]))
                            comparisons+=1;loaded+=int(value[0]>0);adhesive+=int(data.contact.adhesion[index]>0)
                    self.assertGreater(comparisons,0)
                    self.assertGreater(loaded,0)
                    self.assertGreater(adhesive,0)


if __name__=='__main__': unittest.main()
