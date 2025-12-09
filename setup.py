from setuptools import setup, find_packages

setup(
    name="camera-intrinsics-converter",
    version="0.1.0",
    description="Convert camera intrinsics from manufacturer formats to standard YAML format",
    author="windzu",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.20.0",
        "scipy>=1.7.0",
        "pyyaml>=5.4.0",
    ],
    entry_points={
        "console_scripts": [
            "convert-intrinsics=camera_converter.cli:main",
        ],
    },
    python_requires=">=3.7",
)
