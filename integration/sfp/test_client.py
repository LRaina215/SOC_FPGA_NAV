"""Offline address/signature contracts; never opens a serial port."""
import unittest
from sfp_client import inspect_registers


class ClientContracts(unittest.TestCase):
    def bank(self,base,signature):
        values={base:signature,base+4:2,base+8:7,base+0x10:23,
                base+0x18:0x00010083,base+0x20:123}
        writes=[]
        def write(addr,value):
            writes.append((addr,value));values[addr]=value
        return values,lambda addr:values.get(addr,0xdeadbeef),write,writes

    def test_joint_registers_and_read_only_default(self):
        _,read,write,writes=self.bank(0x1300,0x53465032)
        data=inspect_registers(read,write)
        self.assertEqual((data['image'],data['register_base']),('joint',0x1300))
        self.assertEqual(data['telemetry_accepted'],123)
        self.assertNotIn('echoes',data)
        self.assertEqual(writes,[])

    def test_standalone_fallback_and_mapping(self):
        _,read,write,writes=self.bank(0,0x53465031)
        data=inspect_registers(read,write)
        self.assertEqual(data['image'],'standalone')
        self.assertEqual(data['rx_frames'],123)
        self.assertNotIn('telemetry_accepted',data)
        self.assertEqual(writes,[])

    def test_joint_mode_write_readback_and_copper_status(self):
        values,read,write,writes=self.bank(0x1300,0x53465032)
        values[0x1308]=263
        data=inspect_registers(read,write,'joint','sgmii-an',True,sleep=lambda _:None)
        self.assertEqual(writes,[(0x1304,7)])
        self.assertTrue(data['decoded']['partner_phy_link'])

    def test_wrong_image_never_writes(self):
        _,read,write,writes=self.bank(0,0x53465031)
        with self.assertRaises(RuntimeError):
            inspect_registers(read,write,'joint','sgmii-an',sleep=lambda _:None)
        self.assertEqual(writes,[])

    def test_unknown_image_never_writes(self):
        _,read,write,writes=self.bank(0,0x4b28)
        with self.assertRaises(RuntimeError):
            inspect_registers(read,write,mode='sgmii-an',sleep=lambda _:None)
        self.assertEqual(writes,[])

    def test_write_readback_mismatch_rejected(self):
        _,read,_,_=self.bank(0x1300,0x53465032)
        with self.assertRaises(RuntimeError):
            inspect_registers(read,lambda *_:None,'joint','sgmii-an',sleep=lambda _:None)


if __name__=='__main__':unittest.main()
