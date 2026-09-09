"""Small CPU view-cache checks; no assets or device required."""
import gc
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import weakref

import numpy as np
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "quest/app/src/main/python"))
from quest_sim.cpu_views import CpuViewCache


class CpuViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        wp.set_device("cpu")

    def test_reuses_cpu_view_and_skips_only_its_exact_self_assignment(self):
        cache = CpuViewCache()
        array = wp.array([1., 2., 3.], dtype=wp.float32, device="cpu")
        view = cache.numpy(array)
        self.assertIs(cache.numpy(array), view)
        view[1] = 7
        with patch.object(wp.array, "assign", autospec=True) as assign:
            cache.assign(array, view)
            assign.assert_not_called()
        np.testing.assert_array_equal(array.numpy(), [1., 7., 3.])
        replacement = np.array([9., 8., 7.], dtype=np.float32)
        cache.assign(array, replacement)
        np.testing.assert_array_equal(array.numpy(), replacement)
        for dtype in (wp.transform, wp.spatial_vector):
            compound = wp.zeros(2, dtype=dtype, device="cpu")
            self.assertIs(cache.numpy(compound), cache.numpy(compound))

    def test_replacement_rebinds_and_cache_keeps_only_bounded_owners(self):
        cache = CpuViewCache()
        array = wp.array([1.], dtype=wp.float32, device="cpu")
        first = cache.numpy(array)
        owner = weakref.ref(array)
        array = wp.array([2.], dtype=wp.float32, device="cpu")
        self.assertIsNot(cache.numpy(array), first)
        self.assertEqual(cache.numpy(array)[0], 2.)
        del first
        gc.collect()
        self.assertIsNotNone(owner())
        for index in range(20):
            array = wp.array([float(index)], dtype=wp.float32, device="cpu")
            cache.numpy(array)
        gc.collect()
        self.assertLessEqual(cache.diagnostics()["cached_views"], 16)
        self.assertIsNone(owner())

    def test_debug_tape_gradient_and_strided_views_use_ordinary_methods(self):
        cache = CpuViewCache()
        array = wp.array([1., 2.], dtype=wp.float32, device="cpu")
        cached = cache.numpy(array)
        previous = wp.config.launch_array_access_mode
        try:
            wp.config.launch_array_access_mode = wp.config.LaunchArrayAccessMode.STRICT
            self.assertIsNot(cache.numpy(array), cached)
            with patch.object(wp.array, "assign", autospec=True) as assign:
                cache.assign(array, cached)
                assign.assert_called_once()
        finally:
            wp.config.launch_array_access_mode = previous
        with wp.Tape():
            self.assertIsNot(cache.numpy(array), cached)
        gradient = wp.array([1.], dtype=wp.float32, device="cpu", requires_grad=True)
        self.assertIsNot(cache.numpy(gradient), cache.numpy(gradient))
        integers = wp.array([1, 2], dtype=wp.int32, device="cpu")
        self.assertIsNot(cache.numpy(integers), cache.numpy(integers))
        strided = wp.array(np.arange(8, dtype=np.float32).reshape(2, 4), device="cpu").transpose()
        self.assertFalse(strided.is_contiguous)
        self.assertIsNot(cache.numpy(strided), cache.numpy(strided))
        np.testing.assert_array_equal(cache.numpy(strided), strided.numpy())


if __name__ == "__main__":
    unittest.main()
