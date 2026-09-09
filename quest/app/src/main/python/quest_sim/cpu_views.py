"""Bounded views of owned CPU Warp arrays; no incoming JNI array is retained.

Pinned Warp's array.numpy() aliases CPU storage. Reusing that view avoids its
stream/device/interface wrappers. Only assigning that same validated view back
to its owner can omit the redundant copy; all other inputs use Warp.assign.
"""
from collections import OrderedDict

import warp as wp
from warp._src import context as _wp_context


class CpuViewCache:
    _MAX_VIEWS = 16

    def __init__(self):
        self._views = OrderedDict()
        self._hits = 0
        self._builds = 0
        self._self_copies_avoided = 0

    @staticmethod
    def _binding(array):
        runtime = _wp_context.runtime
        if (not isinstance(array, wp.array) or not array.device.is_cpu
                or not array.is_contiguous or array.requires_grad or not array.ptr
                or getattr(array.dtype, "_wp_scalar_type_", array.dtype) is not wp.float32
                or runtime is None or runtime.tape is not None
                or getattr(runtime, "_apic_capture", None) is not None or runtime.captures
                or wp.config.launch_array_access_mode != wp.config.LaunchArrayAccessMode.RELAXED
                or wp.config.verify_autograd_array_access or wp.config.verify_fp
                or wp.config.verify_cuda or wp.config.print_launches):
            return None
        # Identity is insufficient: Warp buffers can be rebound or reshaped.
        return (id(array), array.ptr, array.shape, array.strides, array.dtype, id(array.device))

    def numpy(self, array):
        binding = self._binding(array)
        if binding is None:
            return array.numpy()
        entry = self._views.get(id(array))
        if entry is not None and entry[0] == binding:
            self._views.move_to_end(id(array))
            self._hits += 1
            return entry[2]
        view = array.numpy()
        # Keep the owner, not only a pointer, alive while the view is cached.
        self._views[id(array)] = (binding, array, view)
        self._views.move_to_end(id(array))
        self._builds += 1
        if len(self._views) > self._MAX_VIEWS:
            self._views.popitem(last=False)
        return view

    def assign(self, array, source):
        binding = self._binding(array)
        entry = self._views.get(id(array))
        if binding is not None and entry is not None and entry[0] == binding and entry[2] is source:
            # The caller already wrote into this precise owner's storage.
            # Tapes/gradients/debug paths never reach this shortcut.
            array.mark_init()
            self._self_copies_avoided += 1
            return
        array.assign(source)

    def diagnostics(self):
        return dict(cached_views=len(self._views), max_cached_views=self._MAX_VIEWS,
                    view_hits=self._hits, view_builds=self._builds,
                    self_copies_avoided=self._self_copies_avoided)
