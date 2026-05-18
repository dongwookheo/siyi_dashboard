from glob import glob
from setuptools import find_packages, setup

package_name = "siyi_dashboard"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hdwook",
    maintainer_email="hdwook3918@gmail.com",
    description="Web dashboard backend for SIYI monitoring (ROS 2 node + Tornado WebSocket).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dashboard_node = siyi_dashboard.dashboard_node:main",
            "image_bridge = siyi_dashboard.image_bridge:main",
        ],
    },
)
