"""Reject unsafe routing plans before changing constraints or starting PNR."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from prepare_clock_resource_plan import apply


class ClockPlanTests(unittest.TestCase):
    def test_missing_source_net_and_clock_demotion_preserve_original_cst(self):
        for failure in ('missing_source_net','clock_demotion'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as folder:
                root=Path(folder);cst=root/'g2.cst';cst.write_text('// original IO\n')
                plan=dict(reference_physical_build='reference',promote=[
                    dict(net=f'control{i}',primary_index=4+i,signal_types='SR') for i in range(4)],local=['local'])
                (root/'plan.json').write_text(json.dumps(plan))
                rows=[dict(net=f'control{i}',direct_pin_counts={'RESET':4000}) for i in range(4)]
                rows.append(dict(net='local',direct_pin_counts={'CLK':200} if failure=='clock_demotion' else {'CE':200}))
                nets=['control0','control1','control2','control3','local']
                if failure=='missing_source_net':nets.remove('control3')
                with patch('prepare_clock_resource_plan.audit',return_value=dict(global_nets=rows)),patch(
                    'prepare_clock_resource_plan.read',return_value={'cell':dict(ports={'I':nets})}):
                    with self.assertRaises(AssertionError):apply(root,root,root/'plan.json')
                self.assertEqual(cst.read_text(),'// original IO\n')
                self.assertFalse((root/'clock_resource_plan.json').exists())


if __name__=='__main__':unittest.main()
