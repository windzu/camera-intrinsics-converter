"""
Distortion model conversion and fitting
Converts from rational polynomial model to RadTan (plumb_bob) model
"""

import numpy as np
from scipy.optimize import least_squares
from typing import Tuple, Dict, Any


class DistortionConverter:
    """Convert between different distortion models"""

    def __init__(self, image_width: int, image_height: int):
        self.image_width = image_width
        self.image_height = image_height

    def rational_to_radtan_fit(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        k1: float,
        k2: float,
        p1: float,
        p2: float,
        k3: float,
        k4: float,
        k5: float,
        k6: float,
        num_samples: int = 10000,
    ) -> Dict[str, float]:
        """
        Convert rational polynomial distortion model to RadTan model using fitting

        Rational model:
            x_d = x * (1 + k1*r² + k2*r⁴ + k3*r⁶) / (1 + k4*r² + k5*r⁴ + k6*r⁶) + tangential

        RadTan model:
            x_d = x * (1 + k1*r² + k2*r⁴ + k3*r⁶) + tangential

        Args:
            fx, fy, cx, cy: Intrinsic parameters
            k1-k6, p1, p2: Original rational model coefficients
            num_samples: Number of sample points for fitting

        Returns:
            Dictionary with fitted RadTan coefficients: k1_fit, k2_fit, p1_fit, p2_fit, k3_fit
        """
        # Generate sample points in normalized image coordinates
        sample_points = self._generate_sample_points(num_samples, fx, fy, cx, cy)

        # Calculate distorted points using rational model
        distorted_points = self._apply_rational_distortion(
            sample_points, k1, k2, p1, p2, k3, k4, k5, k6
        )

        # Fit RadTan model to these point correspondences
        fitted_params = self._fit_radtan_model(sample_points, distorted_points)

        # Calculate fitting error
        fitting_error = self._calculate_fitting_error(
            sample_points, distorted_points, fitted_params
        )

        return {
            "k1": fitted_params[0],
            "k2": fitted_params[1],
            "p1": fitted_params[2],
            "p2": fitted_params[3],
            "k3": fitted_params[4],
            "fitting_error_rms": fitting_error,
        }

    def _generate_sample_points(
        self, num_samples: int, fx: float, fy: float, cx: float, cy: float
    ) -> np.ndarray:
        """
        Generate sample points in normalized coordinates
        Uses stratified sampling to ensure good coverage
        """
        # Create a grid that covers the image with higher density near center
        # Convert pixel coordinates to normalized coordinates

        # Use a combination of uniform grid and radial sampling
        sqrt_samples = int(np.sqrt(num_samples * 0.7))

        # Grid sampling
        x_pixel = np.linspace(0, self.image_width, sqrt_samples)
        y_pixel = np.linspace(0, self.image_height, sqrt_samples)
        xx, yy = np.meshgrid(x_pixel, y_pixel)

        # Convert to normalized coordinates
        x_norm_grid = (xx.flatten() - cx) / fx
        y_norm_grid = (yy.flatten() - cy) / fy

        # Radial sampling (for better coverage at different radii)
        num_radial = num_samples - len(x_norm_grid)
        if num_radial > 0:
            # Maximum radius in normalized coordinates
            corners = np.array(
                [
                    [-cx / fx, -cy / fy],
                    [(self.image_width - cx) / fx, -cy / fy],
                    [-cx / fx, (self.image_height - cy) / fy],
                    [(self.image_width - cx) / fx, (self.image_height - cy) / fy],
                ]
            )
            max_radius = np.max(np.sqrt(corners[:, 0] ** 2 + corners[:, 1] ** 2))

            # Sample radii and angles
            radii = np.random.uniform(0, max_radius, num_radial)
            angles = np.random.uniform(0, 2 * np.pi, num_radial)

            x_norm_radial = radii * np.cos(angles)
            y_norm_radial = radii * np.sin(angles)

            x_norm = np.concatenate([x_norm_grid, x_norm_radial])
            y_norm = np.concatenate([y_norm_grid, y_norm_radial])
        else:
            x_norm = x_norm_grid
            y_norm = y_norm_grid

        return np.column_stack([x_norm, y_norm])

    def _apply_rational_distortion(
        self,
        points: np.ndarray,
        k1: float,
        k2: float,
        p1: float,
        p2: float,
        k3: float,
        k4: float,
        k5: float,
        k6: float,
    ) -> np.ndarray:
        """Apply rational polynomial distortion model"""
        x = points[:, 0]
        y = points[:, 1]

        r2 = x**2 + y**2
        r4 = r2**2
        r6 = r2**3

        # Radial distortion (rational model)
        numerator = 1 + k1 * r2 + k2 * r4 + k3 * r6
        denominator = 1 + k4 * r2 + k5 * r4 + k6 * r6
        radial_factor = numerator / denominator

        # Tangential distortion
        xy = x * y
        tangential_x = 2 * p1 * xy + p2 * (r2 + 2 * x**2)
        tangential_y = p1 * (r2 + 2 * y**2) + 2 * p2 * xy

        x_distorted = x * radial_factor + tangential_x
        y_distorted = y * radial_factor + tangential_y

        return np.column_stack([x_distorted, y_distorted])

    def _apply_radtan_distortion(
        self, points: np.ndarray, params: np.ndarray
    ) -> np.ndarray:
        """Apply RadTan distortion model"""
        k1, k2, p1, p2, k3 = params

        x = points[:, 0]
        y = points[:, 1]

        r2 = x**2 + y**2
        r4 = r2**2
        r6 = r2**3

        # Radial distortion
        radial_factor = 1 + k1 * r2 + k2 * r4 + k3 * r6

        # Tangential distortion
        xy = x * y
        tangential_x = 2 * p1 * xy + p2 * (r2 + 2 * x**2)
        tangential_y = p1 * (r2 + 2 * y**2) + 2 * p2 * xy

        x_distorted = x * radial_factor + tangential_x
        y_distorted = y * radial_factor + tangential_y

        return np.column_stack([x_distorted, y_distorted])

    def _fit_radtan_model(
        self, undistorted_points: np.ndarray, distorted_points: np.ndarray
    ) -> np.ndarray:
        """Fit RadTan model parameters using least squares"""

        def residuals(params):
            predicted = self._apply_radtan_distortion(undistorted_points, params)
            diff = predicted - distorted_points
            return diff.flatten()

        # Initial guess: use original k1, k2, p1, p2, k3 as starting point
        # This is passed from outside, but for now use zeros as conservative guess
        initial_params = np.zeros(5)

        # Optimize
        result = least_squares(
            residuals, initial_params, method="lm", verbose=0  # Levenberg-Marquardt
        )

        return result.x

    def _calculate_fitting_error(
        self,
        undistorted_points: np.ndarray,
        target_distorted_points: np.ndarray,
        fitted_params: np.ndarray,
    ) -> float:
        """Calculate RMS fitting error"""
        predicted = self._apply_radtan_distortion(undistorted_points, fitted_params)
        diff = predicted - target_distorted_points
        rms = np.sqrt(np.mean(diff**2))
        return float(rms)

    def check_rational_model_significance(
        self, k4: float, k5: float, k6: float, threshold: float = 0.01
    ) -> Tuple[bool, str]:
        """
        Check if rational model coefficients are significant

        Returns:
            (is_significant, message)
        """
        max_coeff = max(abs(k4), abs(k5), abs(k6))

        if max_coeff > threshold:
            return True, (
                f"Rational model coefficients are significant (max={max_coeff:.4f}). "
                f"Direct conversion may cause noticeable distortion errors."
            )
        else:
            return False, (
                f"Rational model coefficients are small (max={max_coeff:.4f}). "
                f"Direct conversion should be acceptable."
            )
