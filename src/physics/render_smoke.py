"""Headless MuJoCo offscreen render smoke test."""

from __future__ import annotations

import numpy as np
import mujoco


WIDTH = 64
HEIGHT = 48

MODEL_XML = """
<mujoco model="render_smoke">
  <visual>
    <global offwidth="64" offheight="48"/>
    <quality shadowsize="256"/>
  </visual>
  <worldbody>
    <light name="key" pos="0 -3 3" dir="0 1 -1" diffuse="1 1 1"/>
    <camera name="smoke_cam" pos="1.4 -2.0 1.0" xyaxes="1 0 0 0 0.45 1"/>
    <geom name="floor" type="plane" size="1 1 0.05" rgba="0.2 0.3 0.4 1"/>
    <body name="target" pos="0 0 0.2">
      <geom type="box" size="0.18 0.12 0.08" rgba="0.9 0.1 0.05 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def main() -> None:
    gl_context = mujoco.GLContext(WIDTH, HEIGHT)
    gl_context.make_current()

    renderer = None
    try:
        model = mujoco.MjModel.from_xml_string(MODEL_XML)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
        renderer.update_scene(data, camera="smoke_cam")
        pixels = renderer.render()

        expected_shape = (HEIGHT, WIDTH, 3)
        if pixels.shape != expected_shape:
            raise RuntimeError(f"unexpected render shape {pixels.shape}, expected {expected_shape}")

        if pixels.size == 0:
            raise RuntimeError("render returned an empty pixel buffer")

        pixel_min = int(np.min(pixels))
        pixel_max = int(np.max(pixels))
        if pixel_max <= pixel_min:
            raise RuntimeError(
                f"render appears flat: pixel_min={pixel_min} pixel_max={pixel_max}"
            )

        print(
            f"mujoco-render-ok width={WIDTH} height={HEIGHT} "
            f"shape={pixels.shape} pixel_min={pixel_min} pixel_max={pixel_max}"
        )
        print("PASS: DFD-002 base image imports and headless render passed")
    finally:
        if renderer is not None:
            renderer.close()
        gl_context.free()


if __name__ == "__main__":
    main()
