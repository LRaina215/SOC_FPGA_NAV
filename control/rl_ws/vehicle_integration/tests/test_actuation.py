import sys,unittest,itertools,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime'))
from actuation import LegMapper,servo_pulse,wheel_torques
class ActuationTest(unittest.TestCase):
 def test_action_extremes_preserve_closure_and_front_target(self):
  m=LegMapper()
  for h in [.09,.13,.18]:
   for a,b in itertools.product([-1,0,1],repeat=2):
    m.height=h;result=m.update([0,0,h],[a,b,0,a,b,0],.008)
    for side in ['Left','Right']:
     out=result[side];q=m.geometry.solve(side,*out['wheel_yz_m'])
     rear=m.geometry.rear_fk(side,out['back_rad'],q['back_knee'])
     self.assertLess(math.dist(rear,out['wheel_yz_m']),1e-9)
     ref=m.geometry.solve(side,-.03,.0325-h)
     self.assertAlmostEqual(out['front_rad'],ref['front']+a*.03,places=8)
 def test_unreachable_target_rejects(self):
  m=LegMapper();rejected=0
  for h in [.085,.19]:
   for a,b in itertools.product([-1,0,1],repeat=2):
    m.height=h
    try:m.update([0,0,h],[a,b,0,a,b,0],.008)
    except ValueError:rejected+=1
  self.assertGreater(rejected,0)
 def test_height_slew_and_torque_contract(self):
  m=LegMapper();m.update([0,0,.19],[0]*6,.008);self.assertAlmostEqual(m.height,.1309)
  self.assertEqual(wheel_torques([0,0,1,0,0,-1]),{'Left':.1,'Right':-.1})
 def test_missing_calibration_and_limits_reject(self):
  with self.assertRaises(ValueError):servo_pulse(0,{})
  c=dict(zero_joint_rad=0,zero_us=1500,us_per_rad=-500,min_us=1000,max_us=2000,min_joint_rad=-.5,max_joint_rad=.5)
  self.assertEqual(servo_pulse(.25,c),1375)
  with self.assertRaises(ValueError):servo_pulse(.6,c)
if __name__=='__main__':unittest.main()
