"""Deterministic host comparison of explicit Franka contact profiles.

Use normal Session commands only. Completion is separate from manipulation
quality: a failed baseline grasp is neither required nor hidden. No device calls.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import logging
import math
from pathlib import Path
import statistics
import sys
import time
import traceback

PROFILES=('legacy_mesh_v1','five_pads_v1','pad_manipulation_v1')
DT=.005
TARGET=(0.,-math.pi/4,0.,-3*math.pi/4,0.,math.pi/2,math.pi/4)
SETTINGS=dict(physics_dt=DT,control_decimation=2,render_interval=2,floating_base=False,gravity_scale=1.,
              gripper_force_hold=True,gripper_speed_mps=.05,gripper_force_n=5.,initial_base_pose=[0,0,.7,0,0,0,1])
FORCE_EPS=1e-6
MAX_STEPS=3000


def settings_for_mode(mode):
    if mode not in ('force','position'):
        raise ValueError('gripper mode must be force or position')
    return dict(SETTINGS,gripper_force_hold=mode=='force')


def _stats(values):
    values=sorted(values)
    if not values:return dict(count=0,mean=None,p95=None,p99=None,maximum=None)
    return dict(count=len(values),mean=statistics.mean(values),p95=values[math.ceil(len(values)*.95)-1],
                p99=values[math.ceil(len(values)*.99)-1],maximum=values[-1])


def _distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def _finger_contacts(row):
    return [c for c in row['contacts'] if c['finger'] is not None and c['force6'][0]>FORCE_EPS]


def _bilateral(row):return {c['finger'] for c in _finger_contacts(row)}=={0,1}


def _static_force(row,role=None):
    return sum(c['force6'][0] for c in row['contacts'] if c['role'] in ('floor','table','other_static')
               and (role is None or c['role']==role))


def stage_summary(rows,initial_cube,reference_cube_in_hand):
    """Raw metrics; zero-force proximity/constraint rows never count as loaded."""
    if not rows:return None
    all_contacts=[c for r in rows for c in r['contacts'] if c['finger'] is not None]
    loaded=[c for c in all_contacts if c['force6'][0]>FORCE_EPS]
    counts=[[sum(c['finger']==f for c in _finger_contacts(r)) for f in (0,1)] for r in rows]
    normal=[[sum(c['force6'][0] for c in _finger_contacts(r) if c['finger']==f) for f in (0,1)] for r in rows]
    return dict(samples=len(rows),bilateral_samples=sum(_bilateral(r) for r in rows),
        minimum_active_distance_m=min((c['distance_m'] for c in all_contacts),default=None),
        minimum_force_bearing_distance_m=min((c['distance_m'] for c in loaded),default=None),
        peak_finger_point_counts=[max(c[f] for c in counts) for f in (0,1)],
        peak_finger_normal_sum_n=[max(c[f] for c in normal) for f in (0,1)],
        contact_dimensions=sorted({c['condim'] for c in loaded}),
        peak_torsional_torque_abs_nm=max((abs(c['force6'][3]) for c in loaded),default=0.),
        peak_rolling_torque_abs_nm=max((max(abs(c['force6'][4]),abs(c['force6'][5])) for c in loaded),default=0.),
        maximum_cube_displacement_m=max(_distance(r['cube_pose'][:3],initial_cube) for r in rows),
        maximum_cube_in_hand_translation_change_m=max(_distance(r['cube_in_hand'],reference_cube_in_hand) for r in rows),
        maximum_cube_rise_m=max(r['cube_pose'][2]-initial_cube[2] for r in rows),
        final_cube_rise_m=rows[-1]['cube_pose'][2]-initial_cube[2],
        maximum_hand_rise_m=max(r['hand_pose'][2]-rows[0]['hand_pose'][2] for r in rows),
        peak_total_contact_candidates=max(r['ncon'] for r in rows),
        step_wall_us=_stats([r['step_wall_us'] for r in rows]),sample_wall_us=_stats([r['sample_wall_us'] for r in rows]),
        final_cube_pose=rows[-1]['cube_pose'],final_hand_pose=rows[-1]['hand_pose'],
        final_finger_q=rows[-1]['finger_q'],final_finger_target_q=rows[-1]['finger_target_q'])


def quality_checks(stages,table_top,initial_cube):
    """Conservative declared checks, independent of another profile's failure."""
    supported=stages.get('supported_hold',[])[-100:]
    lifted=stages.get('lift_hold',[])[-100:]
    release=stages.get('release',[])[-20:]
    supported_ok=bool(supported) and sum(_bilateral(r) for r in supported)>=.95*len(supported)
    if supported:
        supported_ok=supported_ok and all(_distance(r['cube_pose'][:2],initial_cube[:2])<.025 for r in supported)
    lifted_ok=None if not lifted else (len(lifted)>=100 and all(
        _bilateral(r) and r['cube_pose'][2]-initial_cube[2]>=.03 and _static_force(r)<=.05 for r in lifted))
    final=release[-1] if release else None
    released=bool(release) and len(release)>=20 and all(not _finger_contacts(r) for r in release)
    if final:
        released=released and abs(final['cube_pose'][2]-(table_top+.025))<.01 and _static_force(final,'table')>0
        released=released and all(q>.039 for q in final['finger_target_q']) and all(q>.038 for q in final['finger_q'])
    return dict(supported_bilateral_hold=supported_ok,sustained_unsupported_lift=lifted_ok,released_on_table=released,
        definitions=dict(supported='last100 samples: >=95% bilateral loaded contact and <25mm horizontal displacement',
            lift='every last100 lift-hold samples: >=30mm rise, bilateral loaded contact, <=.05N total static support',
            release='last20 samples no loaded finger contact; open targets/measured fingers; final table contact and center within10mm of table top+25mm'))


def _inverse_point(pose,point,np):
    q=np.asarray(pose[3:],dtype=float);norm=float(np.dot(q,q))
    if not math.isfinite(norm) or norm<1e-12:raise ValueError('invalid measured hand quaternion')
    vector=np.asarray(point,dtype=float)-np.asarray(pose[:3],dtype=float)
    xyz=-q[:3];w=q[3]
    return (vector+2*(w*np.cross(xyz,vector)+np.cross(xyz,np.cross(xyz,vector)))/norm).tolist()


class Sampler:
    def __init__(self,session):
        self.s=session;self.m=session.solver.mj_model;self.d=session.solver.mj_data
        self.mj=session.solver._mujoco;self.np=session.np;self.force=self.np.empty(6)
        self.mapping=session.solver.mjc_body_to_newton.numpy()[0]
        self.cube=session.object_bodies[1]
        self.fingers={session.model.body_label.index(name):i for i,name in enumerate(('panda_leftfinger','panda_rightfinger'))}
        self.names=[self.mj.mj_id2name(self.m,self.mj.mjtObj.mjOBJ_GEOM,i) or f'geom_{i}' for i in range(self.m.ngeom)]

    def sample(self,stage,step_wall_us):
        start=time.perf_counter();s,m,d,np=self.s,self.m,self.d,self.np;records=[]
        for i in range(d.ncon):
            if d.contact.exclude[i]!=0 or d.contact.efc_address[i]<0:continue
            ga,gb=map(int,d.contact.geom[i]);a,b=map(int,self.mapping[m.geom_bodyid[[ga,gb]]])
            if self.cube not in (a,b):continue
            other=b if a==self.cube else a;other_geom=gb if a==self.cube else ga
            self.mj.mj_contactForce(m,d,i,self.force)
            name=self.names[other_geom]
            role='cube_finger' if other in self.fingers else 'table' if other<0 and 'table' in name else 'floor' if other<0 and ('floor' in name or 'ground' in name) else 'other_static' if other<0 else 'other_body'
            record=dict(geom_a=ga,geom_b=gb,body_a=a,body_b=b,name_a=self.names[ga],name_b=self.names[gb],
                finger=self.fingers.get(other),role=role,distance_m=float(d.contact.dist[i]),
                position=d.contact.pos[i].tolist(),frame=d.contact.frame[i].tolist(),
                condim=int(d.contact.dim[i]),friction=d.contact.friction[i].tolist(),force6=self.force.tolist())
            if not all(math.isfinite(v) for v in [record['distance_m'],*record['position'],*record['frame'],*record['friction'],*record['force6']]):
                raise FloatingPointError('nonfinite contact measurement')
            records.append(record)
            if len(records)>256:raise ValueError('cube active contact observation bound exceeded; no samples silently dropped')
        poses=s.state.body_q.numpy();cube=poses[self.cube].tolist();hand=poses[9].tolist()
        row=dict(stage=stage,step=s.step_index,sim_time=s.sim_time,cube_pose=cube,hand_pose=hand,
            cube_in_hand=_inverse_point(hand,cube[:3],np),cube_velocity=s.state.body_qd.numpy()[self.cube].tolist(),
            finger_q=s.joint_positions()[-2:],finger_target_q=s.control.joint_target_q.numpy()[s.arm_q_indices[-2:]].tolist(),
            finger_qd=s.state.joint_qd.numpy()[s.arm_dof_indices[-2:]].tolist(),
            ncon=int(d.ncon),contacts=records,step_wall_us=step_wall_us,sample_wall_us=0.)
        row['sample_wall_us']=(time.perf_counter()-start)*1e6
        return row


def _model_manifest(s):
    m,mj=s.solver.mj_model,s.solver._mujoco
    name=lambda kind,i:mj.mj_id2name(m,getattr(mj.mjtObj,'mjOBJ_'+kind),i)
    fields=('body_mass','body_inertia','body_pos','body_quat','geom_type','geom_size','geom_pos','geom_quat','geom_dataid',
            'mesh_vert','mesh_face','geom_solref','geom_solimp','geom_friction','geom_condim','geom_contype','geom_conaffinity',
            'geom_margin','geom_gap','jnt_actfrcrange','jnt_actfrclimited','actuator_gainprm','actuator_biasprm',
            'eq_type','eq_obj1id','eq_obj2id','eq_data','eq_solref','eq_solimp')
    hashes={key:hashlib.sha256(getattr(m,key).tobytes()).hexdigest() for key in fields}
    return dict(field_sha256=hashes,counts={key:int(getattr(m,key)) for key in ('nbody','ngeom','nmesh','neq','nq','nv','nu')},
        options={key:getattr(m.opt,key) for key in ('timestep','integrator','solver','iterations','ls_iterations','cone','impratio','enableflags','disableflags')},
        bodies=[dict(name=name('BODY',i),mass=float(m.body_mass[i]),inertia=m.body_inertia[i].tolist()) for i in range(m.nbody)],
        geoms=[dict(id=i,name=name('GEOM',i),body=name('BODY',int(m.geom_bodyid[i])),type=int(m.geom_type[i]),size=m.geom_size[i].tolist(),
                    condim=int(m.geom_condim[i]),friction=m.geom_friction[i].tolist(),solref=m.geom_solref[i].tolist(),solimp=m.geom_solimp[i].tolist()) for i in range(m.ngeom)])


def _fixture(Session,assets,profile,legacy_keyword_absent,settings):
    kwargs={} if legacy_keyword_absent else dict(contact_profile=profile)
    s=Session(str(assets),json.dumps(settings),**kwargs)
    for i in range(300):
        from quest_sim.runtime import HOME
        fraction=min(1.,(i+1)/200)
        s.step(DT,[a+(b-a)*fraction for a,b in zip(HOME[:7],TARGET)],0.,True)
    poses=s.state.body_q.numpy();q=poses[9,3:]
    center=.5*(poses[10,:3]+poses[11,:3])+s.np.asarray(s.wp.quat_rotate(s.wp.quat(*q),s.wp.vec3(0,0,.043)))
    top=float(center[2]-.025)
    s.set_environment(json.dumps(dict(version=1,revision=1,enabled=True,colliders=[
        dict(kind='floor',pose=[0,0,-.025,0,0,0,1],half_extents=[3,3,.025]),
        dict(kind='table',pose=[float(center[0])+.27,float(center[1]),top-.025,0,0,0,1],half_extents=[.3,.3,.025])])) )
    s.command(json.dumps(dict(version=1,op='spawn',id=1,pose=[*map(float,center),0,0,0,1],half_extents=[.025]*3)))
    if float(s.model.body_mass.numpy()[s.object_bodies[1]])!=.125:raise ValueError('cube mass changed')
    cube_joint=s.model.joint_label.index('cube_1_free_joint')
    if (s.model.joint_type.numpy()[cube_joint]!=s.newton.JointType.FREE
            or s.model.joint_parent.numpy()[cube_joint]!=-1 or s.solver.mj_model.neq!=1):
        raise ValueError('cube must remain an independent free body with only the existing finger mimic equality')
    # Preserve all existing control/actuator choices while changing only the
    # explicitly selected contact profile.
    expected_effort=[87.]*4+[12.]*3+[20.,20.]
    if s.model.joint_effort_limit.numpy()[s.arm_dof_indices].tolist()!=expected_effort:raise ValueError('actuator effort caps changed')
    if s.model.joint_target_ke.numpy()[s.arm_dof_indices].tolist()!=[400.]*7+[2000.]*2:raise ValueError('position gains changed')
    if s.model.joint_target_kd.numpy()[s.arm_dof_indices].tolist()!=[80.]*7+[100.]*2:raise ValueError('velocity gains changed')
    if any(s.settings[k]!=settings[k] for k in settings if k!='initial_base_pose'):
        raise ValueError('selected controller/step contract changed')
    metadata=json.loads(s.metadata())
    if not legacy_keyword_absent and metadata.get('contact_model',{}).get('profile')!=profile:raise ValueError('requested/effective contact profile mismatch')
    return s,center.tolist(),top,metadata


def run_suite(assets,profile,label,runtime_root,gripper_mode='force'):
    settings=settings_for_mode(gripper_mode)
    report=dict(schema_version=1,completed=False,profile=profile,label=label,gripper_mode=gripper_mode,settings=settings,fixtures=[],
        scope='Host deterministic contact-profile comparison; no device/camera, cross-platform or Quest throughput claim',
        quality_is_separate_from_completion=True,contact_policy='Raw active native cube contacts; exclude==0 and efc_address>=0; loaded normal >1e-6N; no extra forward or state clipping')
    started=time.perf_counter()
    try:
        sys.path.insert(0,str(runtime_root))
        from quest_sim.runtime import Session
        package=Path(inspect.getfile(Session)).parent
        hashes=lambda:{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in package.glob('*.py')}
        report['runtime_sources']=hashes()
        legacy_absent='contact_profile' not in inspect.signature(Session).parameters
        if legacy_absent and profile!='legacy_mesh_v1':raise ValueError('this runtime has no candidate profile support')
        report['legacy_constructor_without_profile_keyword']=legacy_absent
        total_steps=0
        for fixture_name in ('supported','articulated'):
            s,initial,top,metadata=_fixture(Session,assets,profile,legacy_absent,settings);total_steps+=300
            sampler=Sampler(s);manifest=_model_manifest(s);stages={}
            case=dict(name=fixture_name,initial_cube=initial,table_top=top,metadata=metadata,model=manifest,stages=stages)
            report['fixtures'].append(case)
            schedule=[('close',200),('supported_hold',400)]
            if fixture_name=='articulated':schedule += [('lift',200),('lift_hold',100),('lower',200),('returned_hold',100)]
            schedule += [('release',200)]
            for stage,count in schedule:
                rows=[];stages[stage]=dict(rows=rows)
                for i in range(count):
                    if total_steps>=MAX_STEPS:raise ValueError('bounded comparison step budget exceeded')
                    grip=(i+1)/200 if stage=='close' else 0. if stage=='release' else 1.
                    arm=list(TARGET)
                    lift_fraction=(i+1)/count if stage=='lift' else 1. if stage=='lift_hold' else 1.-(i+1)/count if stage=='lower' else 0.
                    arm[1]-=.3*lift_fraction;arm[5]-=.3*lift_fraction
                    t=time.perf_counter();s.step(DT,arm,grip,True);wall=(time.perf_counter()-t)*1e6;total_steps+=1
                    row=sampler.sample(stage,wall);row['commanded_arm']=arm;row['requested_grip']=grip;rows.append(row)
            reference=stages['supported_hold']['rows'][-1]['cube_in_hand']
            for stage,data in stages.items():data['summary']=stage_summary(data['rows'],initial,reference)
            case['quality']=quality_checks({k:v['rows'] for k,v in stages.items()},top,initial)
            case['command_sequence_sha256']=hashlib.sha256(json.dumps(
                [(stage,row['requested_grip'],row['commanded_arm']) for stage,data in stages.items() for row in data['rows']],
                separators=(',',':')).encode()).hexdigest()
            case['model_unchanged_through_motion']=manifest==_model_manifest(s)
            if not case['model_unchanged_through_motion']:raise ValueError('physical model changed during profile trial')
        report['steps']=total_steps
        report['sources_unchanged']=report['runtime_sources']==hashes()
        if not report['sources_unchanged']:raise ValueError('runtime source changed during comparison')
        report['completed']=True
    except Exception as error:
        report['error']=dict(type=type(error).__name__,message=str(error),traceback=traceback.format_exc())
    report['host_elapsed_seconds']=time.perf_counter()-started
    return report


def compare_reports(paths):
    reports=[]
    for path in paths:
        with path.open('rb') as stream:raw=stream.read(128*1024*1024+1)
        if len(raw)>128*1024*1024:raise ValueError('report exceeds byte bound')
        value=json.loads(raw)
        if value.get('schema_version')!=1 or 'fixtures' not in value:raise ValueError('invalid comparison report')
        reports.append(value)
    settings=reports[0]['settings']
    if any(r['settings']!=settings for r in reports):raise ValueError('reports use different controller/step settings')
    rows=[]
    for report in reports:
        for fixture in report['fixtures']:
            rows.append(dict(profile=report['profile'],label=report['label'],completed=report['completed'],fixture=fixture['name'],
                quality=fixture.get('quality'),model=fixture['model'],stages={k:v.get('summary') for k,v in fixture['stages'].items()}))
    reference=reports[0]
    comparisons=[]
    for report in reports[1:]:
        same_commands=True;dynamics=[]
        for a,b in zip(reference['fixtures'],report['fixtures']):
            def commands(f):
                return hashlib.sha256(json.dumps([(stage,row['requested_grip'],row['commanded_arm'])
                    for stage,data in f['stages'].items() for row in data['rows']],separators=(',',':')).encode()).hexdigest()
            same_commands &= a['name']==b['name'] and commands(a)==commands(b)
            invariant_fields=('body_mass','body_inertia','jnt_actfrcrange','jnt_actfrclimited','actuator_gainprm','actuator_biasprm','eq_type','eq_obj1id','eq_obj2id','eq_data')
            dynamics.append(dict(fixture=a['name'],same_mass_inertia_gains_caps_equalities=all(
                a['model']['field_sha256'][k]==b['model']['field_sha256'][k] for k in invariant_fields)))
        if not same_commands or len(reference['fixtures'])!=len(report['fixtures']):
            raise ValueError('reports contain different command schedules/fixtures')
        comparisons.append(dict(reference_profile=reference['profile'],candidate_profile=report['profile'],same_command_schedules=True,invariants=dynamics))
    return dict(schema_version=1,scope='Observed profile results; no baseline failure is required',settings=settings,rows=rows,comparisons=comparisons)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--franka-description-root',type=Path)
    parser.add_argument('--contact-profile',choices=PROFILES,default='legacy_mesh_v1')
    parser.add_argument('--gripper-mode',choices=('force','position'),default='force')
    parser.add_argument('--label',default='comparison')
    parser.add_argument('--runtime-root',type=Path,default=Path(__file__).resolve().parents[2]/'quest/app/src/main/python')
    parser.add_argument('--compare',nargs='+',type=Path,help='Combine existing profile reports, no physics run')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv);args.output.parent.mkdir(parents=True,exist_ok=True)
    logging.getLogger('trimesh').setLevel(logging.ERROR)
    if args.compare:report=compare_reports(args.compare)
    else:
        if args.franka_description_root is None:parser.error('--franka-description-root is required for a run')
        report=run_suite(args.franka_description_root,args.contact_profile,args.label,args.runtime_root,args.gripper_mode)
    args.output.write_text(json.dumps(report,allow_nan=False,separators=(',',':')),encoding='utf-8')
    print(json.dumps(dict(output=str(args.output),completed=report.get('completed'),profile=report.get('profile'),
        results=[dict(fixture=f['name'],quality=f.get('quality')) for f in report.get('fixtures',[])],error=report.get('error'))))
    return 0 if report.get('completed',True) else 1


if __name__=='__main__':raise SystemExit(main())
