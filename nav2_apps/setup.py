from setuptools import find_packages, setup


package_name = "nav2_apps"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yu Ge",
    maintainer_email="liuzi9240@gmail.com",
    description="Checkpoint 12 Nav2 mission applications.",
    license="Apache-2.0",
    tests_require=["pytest"],
    scripts=[
        "scripts/collect_slice1_evidence",
        "scripts/move_shelf_to_ship.py",
    ],
)
