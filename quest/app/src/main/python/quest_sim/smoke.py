"""Run after bootstrap, before model creation, to prove actual CPU JIT works."""
import json
import numpy as np
import warp as wp


@wp.kernel
def _square(values: wp.array[wp.float32]):
    i = wp.tid()
    values[i] = values[i] * values[i]


def run():
    wp.init()
    values = wp.array([1., 2., 3.], dtype=wp.float32, device="cpu")
    wp.launch(_square, dim=3, inputs=[values], device="cpu")
    actual = values.numpy()
    if not np.array_equal(actual, np.array([1., 4., 9.], dtype=np.float32)):
        raise RuntimeError(f"CPU JIT smoke failed: {actual}")
    return json.dumps(dict(cpu_jit=True, result=actual.tolist(), warp=wp.__version__, numpy=np.__version__))
