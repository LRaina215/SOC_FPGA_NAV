import sys,struct,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from read_sensors import decode
from protocol import Packet,encode,decode as packet_decode,validate_reply
class SensorTest(unittest.TestCase):
 def test_signed_endpoints_and_offsets(self):
  raw=struct.pack('<IIIhhhhhhIIHBBB',100000,7,50000,-32768,32767,-1,0,12,-12,9,75000,16383,10,3,9)+bytes.fromhex('123456')+bytes(24)
  result=decode(raw)
  self.assertEqual(result['accel_raw'],(-32768,32767,-1))
  self.assertEqual(result['encoder_frame'],0x563412)
  self.assertTrue(result['imu_ready']);self.assertTrue(result['encoder_fault'])
  self.assertFalse(result['encoder_ready']);self.assertEqual(result['encoder_angle'],16383)
  self.assertEqual(result['imu_age_seconds'],.001)
 def test_request_reply_and_exact_length(self):
  self.assertEqual(packet_decode(encode(Packet(3,7))).payload,b'')
  reply=packet_decode(encode(Packet(0x83,7,bytes(64))))
  self.assertEqual(reply.payload,bytes(64));validate_reply(reply,0x83,7)
  with self.assertRaises(ValueError):decode(bytes(63))
