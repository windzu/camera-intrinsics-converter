"""
Command-line interface for camera intrinsics converter
"""

import argparse
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional

from camera_converter.parsers import SensingParser
from camera_converter.output import YAMLGenerator


def main():
    """Main entry point for the CLI"""
    parser = argparse.ArgumentParser(
        description="Convert camera intrinsics from manufacturer formats to standard YAML format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert Sensing format to YAML
  convert-intrinsics sensing input.txt output.yaml \\
    --width 1920 --height 1080 --camera-name CAM_FRONT
  
  # With calibration date
  convert-intrinsics sensing input.txt output.yaml \\
    --width 1920 --height 1080 --camera-name CAM_FRONT \\
    --calibrated-at "2025-09-25T00:00:00Z"
        """,
    )

    parser.add_argument(
        "format",
        choices=["sensing"],
        help="Input file format (currently only sensing is supported)",
    )

    parser.add_argument("input", type=str, help="Input intrinsics file path")

    parser.add_argument("output", type=str, help="Output YAML file path")

    parser.add_argument(
        "--width", type=int, required=True, help="Image width in pixels"
    )

    parser.add_argument(
        "--height", type=int, required=True, help="Image height in pixels"
    )

    parser.add_argument(
        "--camera-name",
        type=str,
        required=True,
        help="Camera name/identifier (e.g., CAM_FRONT, CAM_LEFT)",
    )

    parser.add_argument(
        "--camera-model",
        type=str,
        default="pinhole",
        choices=["pinhole", "fisheye"],
        help="Camera model type (default: pinhole)",
    )

    parser.add_argument(
        "--calibrated-at",
        type=str,
        help="Calibration timestamp in ISO format (e.g., 2025-09-25T00:00:00Z)",
    )

    parser.add_argument(
        "--verbose", action="store_true", help="Print detailed conversion information"
    )

    args = parser.parse_args()

    try:
        convert_intrinsics(
            format_type=args.format,
            input_path=args.input,
            output_path=args.output,
            image_width=args.width,
            image_height=args.height,
            camera_name=args.camera_name,
            camera_model=args.camera_model,
            calibrated_at=args.calibrated_at,
            verbose=args.verbose,
        )
        print(f"✓ Successfully converted to {args.output}")

    except Exception as e:
        print(f"✗ Error: {e}", file=sys.stderr)
        sys.exit(1)


def convert_intrinsics(
    format_type: str,
    input_path: str,
    output_path: str,
    image_width: int,
    image_height: int,
    camera_name: str,
    camera_model: str = "pinhole",
    calibrated_at: Optional[str] = None,
    verbose: bool = False,
):
    """
    Convert camera intrinsics from manufacturer format to YAML

    Args:
        format_type: Input format type ('sensing')
        input_path: Path to input file
        output_path: Path to output YAML file
        image_width: Image width in pixels
        image_height: Image height in pixels
        camera_name: Camera identifier
        camera_model: Camera model type
        calibrated_at: Optional calibration timestamp
        verbose: Whether to print detailed information
    """
    # Parse input file
    if verbose:
        print(f"Parsing {format_type} format file: {input_path}")

    if format_type == "sensing":
        parser = SensingParser(input_path)
        data = parser.parse()
    else:
        raise ValueError(f"Unsupported format: {format_type}")

    if verbose:
        print(f"  Serial Number: {data.get('serial_number', 'N/A')}")
        print(
            f"  Original intrinsics: fx={data['fx']:.4f}, fy={data['fy']:.4f}, cx={data['cx']:.4f}, cy={data['cy']:.4f}"
        )
        print(f"  Has rational model: {data.get('has_rational_model', False)}")

    # Determine distortion model type and prepare coefficients
    has_rational = data.get("has_rational_model", False)

    if has_rational:
        # Use rational model with all 8 parameters
        distortion_model = "rational"
        distortion_coeffs = {
            "k1": data["k1"],
            "k2": data["k2"],
            "p1": data["p1"],
            "p2": data["p2"],
            "k3": data["k3"],
            "k4": data.get("k4") or 0.0,
            "k5": data.get("k5") or 0.0,
            "k6": data.get("k6") or 0.0,
        }
        if verbose:
            print(f"\nDetected rational model, using 'rational' distortion model")
            print(
                f"  K4={distortion_coeffs['k4']:.6f}, K5={distortion_coeffs['k5']:.6f}, K6={distortion_coeffs['k6']:.6f}"
            )
    else:
        # Use standard RadTan (plumb_bob) model
        distortion_model = "plumb_bob"
        distortion_coeffs = {
            "k1": data["k1"],
            "k2": data["k2"],
            "p1": data["p1"],
            "p2": data["p2"],
            "k3": data["k3"],
        }
        if verbose:
            print(f"\nNo rational model detected, using 'plumb_bob' distortion model")

    # Prepare metadata
    meta = {}

    if calibrated_at:
        meta["calibrated_at"] = calibrated_at

    # Add original parameters as reference
    meta["original_format"] = format_type
    if data.get("serial_number"):
        meta["serial_number"] = data["serial_number"]
    if data.get("rms") is not None:
        meta["original_rms"] = float(data["rms"])

    meta["conversion_method"] = "format_only" if has_rational else "direct_copy"
    meta["converted_at"] = datetime.now().isoformat() + "Z"

    # Generate YAML output
    if verbose:
        print(f"\nGenerating YAML output...")

    generator = YAMLGenerator()
    yaml_content = generator.generate(
        image_width=image_width,
        image_height=image_height,
        camera_name=camera_name,
        intrinsics={
            "fx": data["fx"],
            "fy": data["fy"],
            "cx": data["cx"],
            "cy": data["cy"],
        },
        distortion_coefficients=distortion_coeffs,
        camera_model=camera_model,
        distortion_model=distortion_model,
        meta=meta,
    )

    # Save to file
    generator.save(yaml_content, output_path)

    if verbose:
        print(f"\nOutput saved to: {output_path}")


if __name__ == "__main__":
    main()
