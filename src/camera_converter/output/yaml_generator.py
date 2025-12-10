"""
YAML output generator for camera intrinsics
"""

import yaml
from typing import Dict, Any, Optional
from datetime import datetime


class YAMLGenerator:
    """Generate YAML output in the target format"""

    def __init__(self):
        pass

    def generate(
        self,
        image_width: int,
        image_height: int,
        camera_name: str,
        intrinsics: Dict[str, float],
        distortion_coefficients: Dict[str, float],
        camera_model: str = "pinhole",
        distortion_model: str = "plumb_bob",
        meta: Optional[Dict[str, Any]] = None,
    ) -> str:
        """
        Generate YAML string in the target format

        Args:
            image_width: Image width in pixels
            image_height: Image height in pixels
            camera_name: Camera name/identifier
            intrinsics: Dict with fx, fy, cx, cy
            distortion_coefficients: Dict with k1, k2, p1, p2, k3
            camera_model: Camera model type (pinhole or fisheye)
            distortion_model: Distortion model type
            meta: Optional metadata dictionary

        Returns:
            YAML formatted string
        """
        data = {
            "image_width": image_width,
            "image_height": image_height,
            "camera_name": camera_name,
            "camera_model": camera_model,
            "distortion_model": distortion_model,
            "distortion_coefficients": {
                "k1": float(distortion_coefficients["k1"]),
                "k2": float(distortion_coefficients["k2"]),
                "p1": float(distortion_coefficients["p1"]),
                "p2": float(distortion_coefficients["p2"]),
                "k3": float(distortion_coefficients["k3"]),
            },
            "intrinsics": {
                "fx": float(intrinsics["fx"]),
                "fy": float(intrinsics["fy"]),
                "cx": float(intrinsics["cx"]),
                "cy": float(intrinsics["cy"]),
            },
        }

        # Add metadata if provided
        if meta:
            data["meta"] = meta

        # Custom YAML representation for better formatting
        yaml_str = self._format_yaml(data)

        return yaml_str

    def _format_yaml(self, data: Dict[str, Any]) -> str:
        """Format YAML with comments and proper structure"""
        lines = []

        # Basic info
        lines.append(f"image_width: {data['image_width']}")
        lines.append(f"image_height: {data['image_height']}")
        lines.append(f"camera_name: {data['camera_name']}")
        lines.append("")

        # Camera model with comment
        lines.append(
            f"camera_model: {data['camera_model']}  # 成像模型 pinhole or fisheye"
        )

        # Distortion model with comments
        lines.append("# 针孔：plumb_bob（= RadTan）")
        lines.append("# 鱼眼：equidistant")
        lines.append(f"distortion_model: {data['distortion_model']}")

        # Distortion coefficients
        lines.append("distortion_coefficients:")
        dc = data["distortion_coefficients"]
        lines.append(f"  k1: {dc['k1']}")
        lines.append(f"  k2: {dc['k2']}")
        lines.append(f"  p1: {dc['p1']}  # t1")
        lines.append(f"  p2: {dc['p2']}  # t2")
        lines.append(f"  k3: {dc['k3']}  # 若无就写 0（例如fisheye就是0）")
        lines.append("")

        # Intrinsics
        lines.append("intrinsics:")
        intr = data["intrinsics"]
        lines.append(f"  fx: {intr['fx']}")
        lines.append(f"  fy: {intr['fy']}")
        lines.append(f"  cx: {intr['cx']}")
        lines.append(f"  cy: {intr['cy']}")

        # Metadata
        if "meta" in data and data["meta"]:
            lines.append("")
            lines.append("meta:  # 可选元信息（不影响 CameraInfo 语义）")
            meta = data["meta"]
            for key, value in meta.items():
                if isinstance(value, str):
                    lines.append(f'  {key}: "{value}"')
                elif isinstance(value, dict):
                    lines.append(f"  {key}:")
                    for sub_key, sub_value in value.items():
                        if isinstance(sub_value, str):
                            lines.append(f'    {sub_key}: "{sub_value}"')
                        else:
                            lines.append(f"    {sub_key}: {sub_value}")
                else:
                    lines.append(f"  {key}: {value}")

        return "\n".join(lines) + "\n"

    def save(self, yaml_str: str, output_path: str):
        """Save YAML string to file"""
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(yaml_str)
