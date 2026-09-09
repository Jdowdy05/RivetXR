"""Independent numerical cases for the approved verification state policy."""
import importlib.util
import unittest


class StateLimitTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('tools.verification.state_limits'),
                             'shared state policy and excursion accounting must exist')
        from tools.verification import state_limits
        self.limits = state_limits

    def test_every_observed_substep_counts_raw_excursions_in_both_units(self):
        stats = self.limits.StateLimitStats()
        q = [0.] * 9
        q[0], q[7], q[8] = -2e-6, -2e-7, -3e-7
        original = q[:]
        stats.observe(q, [0.] * 9, [1.] * 9)
        self.assertEqual(q, original)
        stats.observe([0.] * 7 + [-4e-7, 0.], [0.] * 9, [1.] * 9)
        stats.observe([0.] * 9, [0.] * 9, [1.] * 9)
        self.assertEqual(stats.as_dict(), dict(samples_checked=3, strict_excursion_samples=2,
            per_joint_excursion_counts=[1, 0, 0, 0, 0, 0, 0, 2, 1],
            max_excursions=[2e-6, 0., 0., 0., 0., 0., 0., 4e-7, 3e-7], allowance_violations=0))

    def test_allowance_violations_count_substeps_instead_of_joints(self):
        stats = self.limits.StateLimitStats()
        q = [0.] * 7 + [-1.001e-6, -1.001e-6]
        stats.observe(q, [0.] * 9, [1.] * 9)
        self.assertEqual(stats.as_dict()['allowance_violations'], 1)
        with self.assertRaises(ValueError):
            self.limits.validate_statistics(stats.as_dict(), 1)

    def test_bounds_and_states_must_be_finite_nine_joint_vectors(self):
        for q, lower, upper in (([0.] * 8, [0.] * 9, [1.] * 9),
                                 ([float('nan')] * 9, [0.] * 9, [1.] * 9),
                                 ([0.] * 9, [float('-inf')] * 9, [1.] * 9),
                                 ([0.] * 9, [1.] * 9, [0.] * 9)):
            with self.subTest(q=q, lower=lower), self.assertRaises(ValueError):
                self.limits.measure_excursions(q, lower, upper)

    def test_aggregate_cannot_claim_more_excursions_than_omitted_substeps(self):
        observed = self.limits.StateLimitStats()
        for _ in range(1000):
            observed.observe([0.] * 9, [0.] * 9, [1.] * 9)
        aggregate = dict(samples_checked=10000, strict_excursion_samples=10000,
                         per_joint_excursion_counts=[0] * 7 + [10000, 0],
                         max_excursions=[0.] * 7 + [5e-7, 0.], allowance_violations=0)
        with self.assertRaisesRegex(ValueError, 'omitted'):
            self.limits.validate_statistics(aggregate, 10000, observed)
        aggregate['strict_excursion_samples'] = 9000
        aggregate['per_joint_excursion_counts'][7] = 9000
        self.limits.validate_statistics(aggregate, 10000, observed)

    def test_omitted_joint_counts_must_have_a_possible_union(self):
        observed = self.limits.StateLimitStats()
        # Observed finger excursions occur separately. Adding a finger-7 count
        # without adding any excursion substep is impossible in omitted data.
        observed.observe([0.] * 7 + [-5e-7, 0.], [0.] * 9, [1.] * 9)
        observed.observe([0.] * 7 + [0., -5e-7], [0.] * 9, [1.] * 9)
        aggregate = dict(samples_checked=3, strict_excursion_samples=2,
                         per_joint_excursion_counts=[0] * 7 + [2, 1],
                         max_excursions=[0.] * 7 + [5e-7, 5e-7], allowance_violations=0)
        with self.assertRaisesRegex(ValueError, 'omitted'):
            self.limits.validate_statistics(aggregate, 3, observed)
        # The aggregate union also cannot grow without any omitted joint count.
        observed = self.limits.StateLimitStats()
        observed.observe([0.] * 7 + [-5e-7, -5e-7], [0.] * 9, [1.] * 9)
        aggregate['per_joint_excursion_counts'][7] = 1
        aggregate['strict_excursion_samples'] = 2
        with self.assertRaisesRegex(ValueError, 'omitted'):
            self.limits.validate_statistics(aggregate, 3, observed)

    def test_maximum_cannot_increase_without_an_omitted_excursion(self):
        observed = self.limits.StateLimitStats()
        observed.observe([0.] * 7 + [-5e-7, 0.], [0.] * 9, [1.] * 9)
        aggregate = dict(samples_checked=10, strict_excursion_samples=1,
                         per_joint_excursion_counts=[0] * 7 + [1, 0],
                         max_excursions=[0.] * 7 + [6e-7, 0.], allowance_violations=0)
        with self.assertRaisesRegex(ValueError, 'maximum'):
            self.limits.validate_statistics(aggregate, 10, observed)
        aggregate['max_excursions'][7] = 5e-7
        self.limits.validate_statistics(aggregate, 10, observed)


if __name__ == '__main__':
    unittest.main()
