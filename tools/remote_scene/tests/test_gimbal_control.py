from types import SimpleNamespace
import unittest
from tools.remote_scene.gimbal_control import *


class Driver:
    simulation_only=True
    limits=SimpleNamespace(pan_min_rad=-1.,pan_max_rad=1.,tilt_min_rad=-.7,tilt_max_rad=.7,max_rate_rad_s=1.)
    def __init__(self):self.now=1;self.pan=0.;self.tilt=0.;self.goal=None;self.calls=[]
    def hold(self,t):self.advance(t);self.goal=None;self.calls.append(('hold',t))
    def advance(self,t):
        if t<self.now:raise ValueError('reverse driver time')
        if self.goal is not None:self.pan+=min((t-self.now)/1e9,max(0.,self.goal[0]-self.pan))
        self.now=t
    def target(self,p,t,n):self.goal=(p,t);self.calls.append(('target',n))
    def feedback(self):return SimpleNamespace(pan_rad=self.pan,tilt_rad=self.tilt,tracked=True)


class GimbalControlTests(unittest.TestCase):
    def setUp(self):self.driver=Driver();self.s=GimbalSession(self.driver,1,nonce=7);self.seq=0
    def send(self,kind,now,clutch=False,pan=.5,challenge=None):
        self.seq+=1
        return unpack_feedback(self.s.handle(pack_command(Command(7,self.s.challenge if challenge is None else challenge,self.seq,self.seq,kind,clutch,pan,0.,100)),now))
    def test_release_required_and_expiry_holds_at_deadline(self):
        self.assertFalse(self.send(AIM,2,True)['flags']&ARMED)
        self.send(HOLD,3);self.assertTrue(self.send(AIM,4,True)['flags']&ARMED)
        self.s.tick(200_000_004)
        self.assertAlmostEqual(self.driver.pan,.1)
        self.assertFalse(self.s.armed);self.assertTrue(self.s.needs_release)
    def test_old_challenge_cannot_refresh_delayed_motion(self):
        self.send(HOLD,2);old=self.s.challenge
        result=self.send(AIM,CHALLENGE_NS+3,True,challenge=old)
        self.assertFalse(result['flags']&ARMED);self.assertIsNone(self.driver.goal)
    def test_replay_and_limits_fail_closed(self):
        self.send(HOLD,2);self.send(AIM,3,True)
        with self.assertRaises(ValueError):self.send(AIM,4,True,pan=2.)
        self.assertFalse(self.s.armed)
        raw=pack_command(Command(7,self.s.challenge,1,1,HOLD))
        with self.assertRaises(ValueError):self.s.handle(raw,5)
    def test_malformed_command_stops_an_active_lease(self):
        self.send(HOLD,2);self.send(AIM,3,True)
        with self.assertRaises(ValueError):self.s.handle(b'bad',4)
        self.assertFalse(self.s.armed)
        self.assertIsNone(self.driver.goal)
    def test_map_reset_is_separate_and_disarms(self):
        calls=[];self.s.reset_map=lambda:calls.append('reset')
        self.send(HOLD,2);self.send(AIM,3,True);self.send(RESET_MAP,4)
        self.assertEqual(calls,['reset']);self.assertTrue(self.s.needs_release);self.assertFalse(self.s.armed)
    def test_wire_rejects_unknown_or_nonfinite_and_hardware(self):
        raw=bytearray(pack_command(Command(7,1,1,1)));raw[-1]=1
        with self.assertRaises(ValueError):unpack_command(raw)
        with self.assertRaises(ValueError):pack_command(Command(7,1,1,1,AIM,True,float('nan')))
        driver=Driver();driver.simulation_only=False
        with self.assertRaises(ValueError):GimbalSession(driver,1)
    def test_advertised_float32_limit_is_accepted_by_actual_simulator(self):
        from tools.remote_scene.gimbal import SimulatedPanTilt
        driver=SimulatedPanTilt(start_ns=1);session=GimbalSession(driver,1,nonce=7)
        hello=unpack_feedback(session.reply(1))
        released=unpack_feedback(session.handle(pack_command(Command(7,hello['challenge'],1,1,HOLD)),2))
        result=unpack_feedback(session.handle(pack_command(Command(7,released['challenge'],2,2,AIM,True,released['pan_max'],0.)),3))
        self.assertTrue(result['flags']&ARMED)


if __name__=='__main__':unittest.main()
