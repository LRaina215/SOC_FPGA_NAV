"""Contracts for the actual inline RAM16 syntax used in PNR Verilog."""
import unittest
from audit_pin_loads import CELLS, PORTS, INPUTS
from audit_global_muxes import bits


class PhysicalParserTests(unittest.TestCase):
    def test_consecutive_inline_rams_do_not_swallow_following_cells(self):
        source = r'''RAM16SDP4 \ram0  (
 .RAD({\addr [3:0]}),
 .DO({\data [3:0]}));
defparam \ram0 .INIT_0=16'h0000;
RAM16SDP4 \ram1  (
 .RAD({\addr [3:0]}),
 .DO({\data [7:4]}));
LUT2 \decode  (
 .I0(sel),
 .I1(run),
 .F(enable)
);
'''
        cells=CELLS.findall(source)
        self.assertEqual([(kind,name) for kind,name,_ in cells],
                         [('RAM16SDP4',r'\ram0'),('RAM16SDP4',r'\ram1'),('LUT2',r'\decode')])
        self.assertEqual(len(PORTS.findall(cells[0][2])),2)

    def test_vector_and_mux_select_loads(self):
        self.assertEqual(bits(r'{\block/addr [3:0]}'),
                         ['block/addr[3]','block/addr[2]','block/addr[1]','block/addr[0]'])
        self.assertTrue({'RAD','WAD','S0','S1'} <= INPUTS)


if __name__=='__main__':unittest.main()
