import unittest
from generate import check
from road_standard import stop_s


class StandardTests(unittest.TestCase):
    def test_default_and_shoulder_dimensions(self):
        for shoulder, extent in [(False, 22.5), (True, 23.)]:
            c = check(dict(shoulder_enabled=shoulder))
            self.assertEqual(c['junction_radius_m'], extent)
            self.assertEqual(c['spacing_m'], 350.)
            self.assertEqual(c['incoming_lanes'], 3)
            self.assertEqual(c['allowed_turns'], [['left'], ['straight'], ['right']])
            self.assertAlmostEqual((150-6)-(stop_s(150)+.15), 1.)

    def test_incompatible_config_rejected(self):
        for change in [dict(incoming_lanes=4), dict(allowed_turns=[['left','straight'],['straight'],['right']]),
                       dict(allowed_turns=[['uturn'],['straight'],['right']]), dict(lane_width_m=3.),
                       dict(shoulder_enabled='false'), dict(junction_radius_m=22),
                       dict(shoulder_width_m=.5), dict(spacing_m=180), dict(rows=6)]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                check(change)

    def test_roundtrip_normalized_config(self):
        c = check(dict(rows=2, columns=3, shoulder_enabled=True))
        self.assertEqual(check(c), c)

    def test_legacy_t_not_migrated(self):
        c = check(dict(type='t'))
        self.assertNotIn('standard_profile', c)
        self.assertEqual(c['type'], 't')


if __name__ == '__main__':
    unittest.main()
