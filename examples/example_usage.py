#!/usr/bin/env python3
"""
Example script showing how to use the converted YAML file in ROS
"""
import yaml


def load_camera_info(yaml_path):
    """Load camera intrinsics from YAML file"""
    with open(yaml_path, "r") as f:
        calib = yaml.safe_load(f)

    print("Camera Intrinsics Loaded:")
    print(f"  Name: {calib['camera_name']}")
    print(f"  Resolution: {calib['image_width']}x{calib['image_height']}")
    print(f"  Model: {calib['camera_model']}")
    print(f"  Distortion: {calib['distortion_model']}")
    print()

    print("Intrinsic Matrix:")
    intr = calib["intrinsics"]
    print(f"  fx: {intr['fx']}")
    print(f"  fy: {intr['fy']}")
    print(f"  cx: {intr['cx']}")
    print(f"  cy: {intr['cy']}")
    print()

    print("Distortion Coefficients:")
    dc = calib["distortion_coefficients"]
    print(f"  k1: {dc['k1']}")
    print(f"  k2: {dc['k2']}")
    print(f"  p1: {dc['p1']}")
    print(f"  p2: {dc['p2']}")
    print(f"  k3: {dc['k3']}")
    print()

    if "meta" in calib:
        print("Metadata:")
        meta = calib["meta"]
        if "conversion_method" in meta:
            print(f"  Conversion method: {meta['conversion_method']}")
        if "fitting_error_rms" in meta:
            print(f"  Fitting error (RMS): {meta['fitting_error_rms']:.6f}")
        if "serial_number" in meta:
            print(f"  Serial number: {meta['serial_number']}")

    return calib


def create_ros_camera_info(calib):
    """
    Create ROS CameraInfo message structure
    (This is a dictionary representation, in real ROS use sensor_msgs/CameraInfo)
    """
    camera_info = {
        "width": calib["image_width"],
        "height": calib["image_height"],
        "distortion_model": calib["distortion_model"],
        "D": [],  # Distortion coefficients
        "K": [],  # Intrinsic matrix (3x3, row-major)
        "R": [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],  # Rectification matrix (identity)
        "P": [],  # Projection matrix (3x4)
    }

    # Distortion coefficients (k1, k2, p1, p2, k3)
    dc = calib["distortion_coefficients"]
    camera_info["D"] = [dc["k1"], dc["k2"], dc["p1"], dc["p2"], dc["k3"]]

    # Intrinsic matrix K
    intr = calib["intrinsics"]
    camera_info["K"] = [
        intr["fx"],
        0.0,
        intr["cx"],
        0.0,
        intr["fy"],
        intr["cy"],
        0.0,
        0.0,
        1.0,
    ]

    # Projection matrix P (for monocular camera, P = K [I|0])
    camera_info["P"] = [
        intr["fx"],
        0.0,
        intr["cx"],
        0.0,
        0.0,
        intr["fy"],
        intr["cy"],
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    return camera_info


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python example_usage.py <yaml_file>")
        print("\nExample:")
        print("  python example_usage.py output_test.yaml")
        sys.exit(1)

    yaml_path = sys.argv[1]

    # Load calibration
    calib = load_camera_info(yaml_path)

    # Create ROS CameraInfo
    print("\nROS CameraInfo message structure:")
    camera_info = create_ros_camera_info(calib)

    print(f"  width: {camera_info['width']}")
    print(f"  height: {camera_info['height']}")
    print(f"  distortion_model: '{camera_info['distortion_model']}'")
    print(f"  D: {camera_info['D']}")
    print(f"  K: {camera_info['K']}")
    print(f"  R: {camera_info['R']}")
    print(f"  P: {camera_info['P']}")

    print("\n✓ Ready to use in ROS!")
