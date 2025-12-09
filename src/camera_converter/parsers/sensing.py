"""
Sensing camera intrinsics file parser
"""

import re
from typing import Dict, Any, Optional


class SensingParser:
    """Parser for Sensing camera intrinsics format"""

    def __init__(self, filepath: str):
        self.filepath = filepath
        self.data: Dict[str, Any] = {}

    def parse(self) -> Dict[str, Any]:
        """
        Parse Sensing format intrinsics file

        Returns:
            Dictionary containing parsed intrinsics data
        """
        with open(self.filepath, "r", encoding="utf-8") as f:
            content = f.read()

        # Parse each field
        self.data = {
            "serial_number": self._extract_field(content, r"SN码[:：](.+?)(?:\n|$)"),
            "fx": self._extract_float(content, r"FX[:：](.+?)(?:\n|$)"),
            "fy": self._extract_float(content, r"FY[:：](.+?)(?:\n|$)"),
            "cx": self._extract_float(content, r"CX[:：](.+?)(?:\n|$)"),
            "cy": self._extract_float(content, r"CY[:：](.+?)(?:\n|$)"),
            "k1": self._extract_float(content, r"K1[:：](.+?)(?:\n|$)"),
            "k2": self._extract_float(content, r"K2[:：](.+?)(?:\n|$)"),
            "p1": self._extract_float(content, r"P1[:：](.+?)(?:\n|$)"),
            "p2": self._extract_float(content, r"P2[:：](.+?)(?:\n|$)"),
            "k3": self._extract_float(content, r"K3[:：](.+?)(?:\n|$)"),
            "k4": self._extract_float(content, r"K4[:：](.+?)(?:\n|$)"),
            "k5": self._extract_float(content, r"K5[:：](.+?)(?:\n|$)"),
            "k6": self._extract_float(content, r"K6[:：](.+?)(?:\n|$)"),
            "rms": self._extract_float(content, r"RMS[:：](.+?)(?:\n|$)"),
        }

        # Validate required fields
        self._validate()

        return self.data

    def _extract_field(self, content: str, pattern: str) -> Optional[str]:
        """Extract a field using regex pattern"""
        match = re.search(pattern, content)
        if match:
            return match.group(1).strip()
        return None

    def _extract_float(self, content: str, pattern: str) -> Optional[float]:
        """Extract a float field using regex pattern"""
        value = self._extract_field(content, pattern)
        if value is not None:
            try:
                return float(value)
            except ValueError:
                return None
        return None

    def _validate(self):
        """Validate that all required fields are present"""
        required_fields = ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"]
        missing = [f for f in required_fields if self.data.get(f) is None]

        if missing:
            raise ValueError(
                f"Missing required fields in Sensing file: {', '.join(missing)}"
            )

        # Check for rational model coefficients
        rational_fields = ["k4", "k5", "k6"]
        has_rational = any(self.data.get(f) is not None for f in rational_fields)

        if has_rational:
            self.data["has_rational_model"] = True
        else:
            self.data["has_rational_model"] = False
