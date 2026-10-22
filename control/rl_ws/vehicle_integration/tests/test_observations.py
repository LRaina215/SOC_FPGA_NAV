import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime'))
from observations import GravityEstimator,ObservationHistory,quantize
class ObservationsTest(unittest.TestCase):
 def test_static_and_rotating_gravity(self):
  f=GravityEstimator(np.eye(3),[0,0,0]);_,g=f.update([0,0,10920],[0,0,0],0);np.testing.assert_allclose(g,[0,0,-1])
  # Positive body-X rotation makes downward gravity point toward body-Y negative.
  for i in range(1,101):_,g=f.update([0,0,0],[939,0,0],i*500000)
  self.assertAlmostEqual(g[1],-np.sin(1),places=3);self.assertAlmostEqual(g[2],-np.cos(1),places=3)
 def test_tick_wrap_and_duplicate(self):
  f=GravityEstimator(np.eye(3),[0,0,0]);f.update([0,0,10920],[0,0,0],0xffffff00)
  f.update([0,0,10920],[0,0,0],499744)
  with self.assertRaises(ValueError):f.update([0,0,10920],[0,0,0],499744)
 def test_history_contains_latest_unclipped_frame(self):
  h=ObservationHistory();a=[.5,-.5,.25,.75,-.75,-.25]
  obs,hist=h.update([100,0,0],[0,0,-1],[0,0,.1305],[2,-3],a)
  self.assertEqual(obs[0],100);self.assertEqual(hist[0],130)
  np.testing.assert_allclose(obs[9:15],[.015,-.015,0,.0225,-.0225,0]);np.testing.assert_allclose(obs[15:21],[0,0,.3,0,0,-.45])
  obs,hist=h.update([0,0,0],[0,0,-1],[0,0,.1305],[0,0],[0]*6)
  self.assertEqual(hist[0],130);self.assertEqual(hist[-27],0)
 def test_quantization_ties_and_overflow(self):
  np.testing.assert_array_equal(quantize(np.array([.5,1.5,-.5,-1.5])/4096),[0,2,0,-2])
  with self.assertRaises(ValueError):quantize([3000])
if __name__=='__main__':unittest.main()
