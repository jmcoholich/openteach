import os
from pathlib import Path

from setuptools import find_packages, setup

try:
    from pybind11.setup_helpers import Pybind11Extension, build_ext
except ImportError as exc:
    raise RuntimeError(
        "Building GeoFIK bindings requires pybind11. Install with `pip install pybind11`, "
        "or run `pip install -e .` so pyproject.toml can install build dependencies."
    ) from exc


ROOT = Path(__file__).parent.resolve()


def eigen_include_dirs():
    candidates = [
        os.environ.get("EIGEN3_INCLUDE_DIR"),
        "/usr/include/eigen3",
        "/usr/local/include/eigen3",
        str(ROOT / "third_party" / "eigen"),
    ]
    return [path for path in candidates if path and Path(path).exists()]


ext_modules = [
    Pybind11Extension(
        "openteach.ik._geofik",
        [
            "openteach/ik/bindings/geofik_pybind.cpp",
            "third_party/GeoFIK/geofik.cpp",
        ],
        include_dirs=[
            "third_party/GeoFIK",
            *eigen_include_dirs(),
        ],
        cxx_std=17,
        extra_compile_args=["-O3"],
    )
]


setup(
    name="open-teach",
    version="1.0.0",
    packages=find_packages(),
    description="Open-Teach:VR Teleoperation for Robotic Manipulation",
    install_requires=["numpy"],
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
