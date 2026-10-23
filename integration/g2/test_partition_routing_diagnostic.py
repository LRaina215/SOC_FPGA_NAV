"""Regression for the actual Gowin split-line module-header terminator."""
import unittest

from partition_routing_diagnostic import stub_definition
from synthesis_connectivity import DECL, CELL


class StubHeaderTests(unittest.TestCase):
    def test_header_does_not_swallow_internal_declarations_or_first_cell(self):
        body = '''a, q
)
;
input a;
output [1:0] q;
wire internal;
  LUT2 old_cell (
    .I0(a), .I1(internal), .F(q[0])
);
'''
        for text in (body, body.replace(')\n;', ');')):
            actual, count = stub_definition('slf_match', text)
            self.assertEqual(count, 2)
            self.assertNotIn('old_cell', actual)
            self.assertNotIn('internal', actual)
            self.assertEqual([m[1] for m in DECL.finditer(actual)], ['input', 'output'])
            self.assertEqual([m[1] for m in CELL.finditer(actual)], ['GND', 'GND'])


if __name__ == '__main__':
    unittest.main()
