from setuptools import find_packages, setup

package_name = "shelf_detection_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot Developer Masterclass",
    maintainer_email="student@example.com",
    description="Detection and bounded stepwise shelf attach service for Checkpoint 12.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "shelf_detection_server = shelf_detection_server.server:main",
            "shelf_geometry_observer = shelf_detection_server.geometry_observer:main",
        ],
    },
)
