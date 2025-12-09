"""
Simple test to verify the conversion pipeline
"""
import os
import sys
import tempfile
import yaml

# Add src to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from camera_converter.parsers import SensingParser
from camera_converter.converters import DistortionConverter
from camera_converter.output import YAMLGenerator


def test_sensing_parser():
    """Test Sensing format parser"""
    print("Testing Sensing parser...")
    
    # Use the example file
    parser = SensingParser('data/sensing.txt')
    data = parser.parse()
    
    assert data['fx'] == 1188.6918066682
    assert data['fy'] == 1188.8881856981
    assert data['cx'] == 958.1162131187
    assert data['cy'] == 770.3201938023
    assert data['serial_number'] == 'H100F1A-H09150733'
    assert data['has_rational_model'] == True
    
    print("  ✓ Sensing parser test passed")


def test_distortion_converter():
    """Test distortion model conversion"""
    print("Testing distortion converter...")
    
    converter = DistortionConverter(1920, 1080)
    
    # Test with sample parameters
    result = converter.rational_to_radtan_fit(
        fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
        k1=0.1, k2=-0.05, p1=0.001, p2=0.001, k3=0.01,
        k4=0.1, k5=-0.05, k6=0.01,
        num_samples=1000  # Use fewer samples for faster testing
    )
    
    assert 'k1' in result
    assert 'k2' in result
    assert 'p1' in result
    assert 'p2' in result
    assert 'k3' in result
    assert 'fitting_error_rms' in result
    
    print(f"  Fitting error: {result['fitting_error_rms']:.6f}")
    print("  ✓ Distortion converter test passed")


def test_yaml_generator():
    """Test YAML output generation"""
    print("Testing YAML generator...")
    
    generator = YAMLGenerator()
    
    yaml_str = generator.generate(
        image_width=1920,
        image_height=1080,
        camera_name='TEST_CAM',
        intrinsics={'fx': 1000.0, 'fy': 1000.0, 'cx': 960.0, 'cy': 540.0},
        distortion_coefficients={'k1': 0.1, 'k2': -0.05, 'p1': 0.001, 'p2': 0.001, 'k3': 0.01},
        meta={'test': 'value'}
    )
    
    # Parse the generated YAML
    data = yaml.safe_load(yaml_str)
    
    assert data['image_width'] == 1920
    assert data['image_height'] == 1080
    assert data['camera_name'] == 'TEST_CAM'
    assert data['intrinsics']['fx'] == 1000.0
    assert data['distortion_coefficients']['k1'] == 0.1
    assert data['meta']['test'] == 'value'
    
    print("  ✓ YAML generator test passed")


def test_full_pipeline():
    """Test the complete conversion pipeline"""
    print("Testing full pipeline...")
    
    # Parse
    parser = SensingParser('data/sensing.txt')
    data = parser.parse()
    
    # Convert
    converter = DistortionConverter(1920, 1080)
    fitted_result = converter.rational_to_radtan_fit(
        fx=data['fx'], fy=data['fy'], cx=data['cx'], cy=data['cy'],
        k1=data['k1'], k2=data['k2'], p1=data['p1'], p2=data['p2'], k3=data['k3'],
        k4=data['k4'], k5=data['k5'], k6=data['k6'],
        num_samples=5000
    )
    
    # Generate YAML
    generator = YAMLGenerator()
    yaml_str = generator.generate(
        image_width=1920,
        image_height=1080,
        camera_name='CAM_FRONT',
        intrinsics={'fx': data['fx'], 'fy': data['fy'], 'cx': data['cx'], 'cy': data['cy']},
        distortion_coefficients={
            'k1': fitted_result['k1'],
            'k2': fitted_result['k2'],
            'p1': fitted_result['p1'],
            'p2': fitted_result['p2'],
            'k3': fitted_result['k3'],
        },
        meta={
            'serial_number': data['serial_number'],
            'fitting_error_rms': fitted_result['fitting_error_rms']
        }
    )
    
    # Save and load back
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(yaml_str)
        temp_path = f.name
    
    try:
        with open(temp_path, 'r') as f:
            loaded = yaml.safe_load(f)
        
        assert loaded['camera_name'] == 'CAM_FRONT'
        assert loaded['meta']['serial_number'] == 'H100F1A-H09150733'
        
        print(f"  Fitting error: {loaded['meta']['fitting_error_rms']:.6f}")
        print("  ✓ Full pipeline test passed")
    finally:
        os.unlink(temp_path)


if __name__ == '__main__':
    print("Running tests...\n")
    
    try:
        test_sensing_parser()
        test_distortion_converter()
        test_yaml_generator()
        test_full_pipeline()
        
        print("\n✓ All tests passed!")
        sys.exit(0)
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
