from setuptools import find_packages, setup

package_name = "rover_exploration"

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
    maintainer="deadend",
    maintainer_email="deadend@example.com",
    description="Autonomous rock-aware exploration and image collection for the Mars rover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "visual_rock_explorer_node = rover_exploration.visual_rock_explorer_node:main",
        ],
    },
)
