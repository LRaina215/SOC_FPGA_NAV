"""Port alias regressions found in the actual CPU/SerDes frozen netlist."""
from pathlib import Path
import tempfile
import unittest

from synthesis_connectivity import read


class EscapedPortTests(unittest.TestCase):
    def test_named_escaped_scalar_and_vector_ports_are_bound(self):
        source = r'''module lane (
  \g_lane[0].we ,
  \g_lane[0].wd ,
  q
);
input \g_lane[0].we ;
input [1:0] \g_lane[0].wd ;
output q;
  LUT3 select (
    .I0(\g_lane[0].we ),
    .I1(\g_lane[0].wd [0]),
    .I2(\g_lane[0].wd [1]),
    .F(q)
);
endmodule
module g2_top (
  enable,
  data,
  result
);
input enable;
input [1:0] data;
output result;
  lane u_ram (
    .\g_lane[0].we (enable),
    .\g_lane[0].wd ({data[0], data[1]}),
    .q(result)
);
endmodule
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'test.vg'
            path.write_text(source, encoding='utf-8')
            actual = read(path)
        self.assertEqual(actual['u_ram/select']['ports'],
                         dict(I0=['enable'], I1=['data[1]'], I2=['data[0]'], F=['result']))


if __name__ == '__main__':
    unittest.main()
