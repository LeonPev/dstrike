from __future__ import annotations

import numpy as np

from src.physics.terrain_generator import bilinear_sample


def test_bilinear_sample_uses_canonical_image_row_order() -> None:
    values = np.array(
        [[1.0, 2.0], [3.0, 4.0]],
        dtype=np.float32,
    )

    assert bilinear_sample(values, -1.0, 1.0, 2.0) == 1.0
    assert bilinear_sample(values, 1.0, 1.0, 2.0) == 2.0
    assert bilinear_sample(values, -1.0, -1.0, 2.0) == 3.0
    assert bilinear_sample(values, 1.0, -1.0, 2.0) == 4.0
