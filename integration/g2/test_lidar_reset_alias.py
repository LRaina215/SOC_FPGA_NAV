"""Actual frozen-netlist regression: reset aliases must not hide CDC changes."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from audit_routing_topology import primitive_directions
from revalidate_lidar_cdc import definitions, revalidate
from synthesis_connectivity import read

BASE = Path('F:/nav_integration_g2_20261023/sfp_clockpriority_pnr4')
NEW = Path('F:/nav_integration_g2_20261023/sfp_lane2_bankselect128_syn6')
LIB = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')


class LidarResetAlias(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (NEW/'impl/gwsynthesis/g2_top.vg').exists():
            raise unittest.SkipTest('requires the preserved actual Lane2 synthesis')
        cls.before = read(BASE/'g2_top.vg')
        cls.after = read(NEW/'impl/gwsynthesis/g2_top.vg')
        cls.old_definitions = definitions(BASE/'g2_top.vg')
        cls.new_definitions = definitions(NEW/'impl/gwsynthesis/g2_top.vg')
        cls.reference = json.loads((BASE/'cdc_reference_audit.json').read_text())
        cls.directions = primitive_directions(LIB)

    def validate(self, cells=None, modules=None):
        return revalidate(self.before, cells or self.after, self.reference,
                          self.directions, self.old_definitions, modules or self.new_definitions)

    def change_cell(self, name, pin, value):
        cells = dict(self.after)
        cells[name] = deepcopy(cells[name])
        cells[name]['ports'][pin] = value
        return cells

    def test_actual_alias_and_all_26_crossings(self):
        result = self.validate()
        self.assertEqual(result['changed_reset_pin_connections'], 36)
        self.assertEqual(result['paths'], 26)
        self.assertEqual(len(result['proved_reset_aliases']), 1)

    def test_changed_first_data_rejected(self):
        name = self.reference['paths'][0]['first_register']
        with self.assertRaisesRegex(AssertionError, 'non-reset connection'):
            self.validate(self.change_cell(name,'D',['GND']))

    def test_changed_first_clock_rejected(self):
        name = self.reference['paths'][0]['first_register']
        with self.assertRaisesRegex(AssertionError, 'non-reset connection'):
            self.validate(self.change_cell(name,'CLK',['clk50_d_wrong']))

    def test_extra_first_q_load_rejected(self):
        row = self.reference['paths'][0]
        cells = dict(self.after)
        extra = deepcopy(cells[row['second_register']])
        extra['name'] = 'test_unexpected_load'
        extra['ports']['Q'] = ['test_unused_output']
        cells[extra['name']] = extra
        with self.assertRaisesRegex(AssertionError, 'unexpected loads'):
            self.validate(cells)

    def test_reset_driver_input_change_rejected(self):
        with self.assertRaisesRegex(AssertionError, 'reset driver input changed'):
            self.validate(self.change_cell('u_lrr/r2_s1','CE',['GND']))

    def test_reset_initial_value_change_rejected(self):
        modules = dict(self.new_definitions)
        body = modules['nav_rst_sync_0']
        self.assertIn("r2_s1.INIT=1'b1",body)
        modules['nav_rst_sync_0'] = body.replace("r2_s1.INIT=1'b1", "r2_s1.INIT=1'b0",1)
        with self.assertRaisesRegex(AssertionError, 'reset module/INIT changed'):
            self.validate(modules=modules)


if __name__ == '__main__':unittest.main()
