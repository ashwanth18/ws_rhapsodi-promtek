"""D455 as two RTX cameras (depth + colour) published over the ROS 2 bridge.

Depth goes out as Isaac's 32FC1 on ``/isaac/camera/depth/*``; the native
``depth_to_mm_node`` turns it into the driver's 16UC1 ``/camera/depth/*``.
Colour is published directly on the driver's topics.
"""

from __future__ import annotations

import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, UsdGeom

from isaac_twin.cell import d455_link_to_optical

# ROS optical (z forward, y down) -> USD camera (looks down -z, y up).
_USD_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0, 1.0])
_H_APERTURE_MM = 20.955

STREAMS = {
    "depth": {
        "type": "depth",
        "frame_id": "camera_depth_optical_frame",
        "image_topic": "isaac/camera/depth/image_raw",
        "info_topic": "isaac/camera/depth/camera_info",
    },
    "color": {
        "type": "rgb",
        "frame_id": "camera_color_optical_frame",
        "image_topic": "camera/color/image_raw",
        "info_topic": "camera/color/camera_info",
    },
}


def _set_matrix(prim, t: np.ndarray) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(t.T.tolist()))


def _camera(stage, path: str, base_to_optical: np.ndarray, width: int, height: int, fx: float):
    cam = UsdGeom.Camera.Define(stage, path)
    _set_matrix(cam.GetPrim(), base_to_optical @ _USD_FROM_OPTICAL)
    cam.GetProjectionAttr().Set("perspective")
    cam.GetHorizontalApertureAttr().Set(_H_APERTURE_MM)
    cam.GetVerticalApertureAttr().Set(_H_APERTURE_MM * height / width)
    cam.GetFocalLengthAttr().Set(fx * _H_APERTURE_MM / width)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 20.0))
    return cam


def add_d455(stage, base_to_camera_link: np.ndarray, cfg: dict, render_hz: float, root: str = "/World/D455") -> None:
    UsdGeom.Xform.Define(stage, root)
    optical = d455_link_to_optical()
    keys = og.Controller.Keys
    for name, stream in STREAMS.items():
        spec = cfg[name]
        cam_path = f"{root}/{name}"
        _camera(stage, cam_path, base_to_camera_link @ optical[name], spec["width"], spec["height"], spec["fx"])
        skip = max(0, int(round(render_hz / float(spec["rate_hz"]))) - 1)
        og.Controller.edit(
            {"graph_path": f"{root}_{name}_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("Image", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "Image.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "Info.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "Image.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "Info.inputs:renderProductPath"),
                    ("Context.outputs:context", "Image.inputs:context"),
                    ("Context.outputs:context", "Info.inputs:context"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(cam_path)]),
                    ("RenderProduct.inputs:width", int(spec["width"])),
                    ("RenderProduct.inputs:height", int(spec["height"])),
                    ("Image.inputs:type", stream["type"]),
                    ("Image.inputs:topicName", stream["image_topic"]),
                    ("Image.inputs:frameId", stream["frame_id"]),
                    ("Image.inputs:frameSkipCount", skip),
                    ("Info.inputs:topicName", stream["info_topic"]),
                    ("Info.inputs:frameId", stream["frame_id"]),
                    ("Info.inputs:frameSkipCount", skip),
                ],
            },
        )
