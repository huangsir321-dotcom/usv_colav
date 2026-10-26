import unittest
from usv_colav.geometry import NM_TO_M
from usv_colav.ccs_reference import lookup_annex_b_sda_m, SDA_TYPE_COEFFICIENTS
from usv_colav.models import Vessel
from usv_colav.scenarios import build_scenario
from usv_colav.simulator import SimConfig, USVSimulator

class CCSReferenceTests(unittest.TestCase):
    def test_annex_b_example_value(self):
        # Informative Annex B.1 example: 100 m bulk carrier vs 100 m target, head-on.
        value = lookup_annex_b_sda_m('head_on', 100)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value, 0.40 * NM_TO_M, places=5)
        self.assertAlmostEqual(SDA_TYPE_COEFFICIENTS['container'], 1.33)

    def test_per_target_sda_overrides_default(self):
        own, targets, note = build_scenario('head_on', num_targets=1)
        targets[0].sda_nm = 0.55
        sim = USVSimulator(own, targets, SimConfig(sda_nm=0.30), note)
        self.assertAlmostEqual(sim.metrics()[0]['sda_nm'], 0.55)

if __name__ == '__main__':
    unittest.main()
