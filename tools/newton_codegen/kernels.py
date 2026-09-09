"""Small host-authored Warp kernels included in the precompiled bundle."""
import warp as wp


@wp.kernel(enable_backward=False)
def add_external_force(external: wp.array[float], total: wp.array[float]):
    index = wp.tid()
    total[index] = total[index] + external[index]
