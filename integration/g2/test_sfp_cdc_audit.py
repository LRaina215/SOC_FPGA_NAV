"""Negative checks for explicit CDC auditing and hierarchy bit aliases."""
import copy,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from audit_sfp_cdc import GROUPS,audit,sdc_fragment
from synthesis_connectivity import read


class SfpAuditTests(unittest.TestCase):
    def fixture(self):
        cells={}
        prefixes=['u_nav/u_neth/u_rxf/wp_g_r1_','u_nav/u_neth/u_rxf/rp_g_w1_',
                  'u_nav/u_neth/u_txf/wp_g_r1_','u_nav/u_neth/u_txf/rp_g_w1_',
                  'u_nav/u_neth/u_fs/u_s/s1_']
        for (group,_,count,src,dst,_),prefix in zip(GROUPS,prefixes):
            for i in range(count):
                first=f'{prefix}{i}_s1'
                second=first.replace('/wp_g_r1_','/wp_g_r2_').replace('/rp_g_w1_','/rp_g_w2_').replace('/s1_','/s2_')
                for name,clk,d,q in [(group+f'/source{i}',src,'input',first+'in'),
                                      (first,dst,first+'in',first+'out'),(second,dst,first+'out',second+'out')]:
                    cells[name]=dict(name=name,kind='DFFCE',ports=dict(CLK=[clk],D=[d],Q=[q],CE=['VCC']))
        return cells

    def check(self,cells):
        with patch('synthesis_connectivity.read',return_value=cells):return audit(Path('fixture.vg'),True)

    def test_valid_and_corrupted_crossings(self):
        cells=self.fixture();result=self.check(cells)
        self.assertTrue(result['passed']);self.assertEqual(sum(l.startswith('set_') for l in sdc_fragment(result).splitlines()),102)
        first='u_nav/u_neth/u_rxf/wp_g_r1_0_s1'
        for pin,value in [('CLK',['clk50_wrong']),('CE',['variable_enable']),('D',['missing_driver'])]:
            bad=copy.deepcopy(cells);bad[first]['ports'][pin]=value
            self.assertFalse(self.check(bad)['passed'])
        bad=copy.deepcopy(cells);bad['extra_load']=dict(name='extra_load',kind='LUT2',ports={'I0':cells[first]['ports']['Q']})
        self.assertFalse(self.check(bad)['passed'])
        bad=copy.deepcopy(cells);del bad[first];self.assertFalse(self.check(bad)['passed'])

    def test_alias_bus_order(self):
        source='''module g2_top (clk, a);
input clk;
input [1:0] a;
wire [1:0] q;
  child u (
    .clk(clk),
    .d({a[0],a[1]}),
    .q(q)
);
endmodule
module child (clk,d,q);
input clk;
input [1:0] d;
output [1:0] q;
  DFF f (
    .CLK(clk),
    .D(d[1]),
    .Q(q[0])
);
endmodule
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'fixture.vg';p.write_text(source)
            ports=read(p)['u/f']['ports']
            self.assertEqual(ports,{'CLK':['clk'],'D':['a[0]'],'Q':['q[0]']})
            p.write_text(source.replace('wire [1:0] q;','wire [1:0] q;\nassign q=a;'))
            with self.assertRaises(AssertionError):read(p)


if __name__=='__main__':unittest.main()
