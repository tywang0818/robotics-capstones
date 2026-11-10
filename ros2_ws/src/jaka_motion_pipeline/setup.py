from glob import glob
from setuptools import find_packages, setup


package_name = "jaka_motion_pipeline"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JAKA Planning Team",
    maintainer_email="student@example.com",
    description="Automated planning-scene and motion-planning pipeline for JAKA A5.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "motion_planner = jaka_motion_pipeline.motion_planner_node:main",
            "example_environment = jaka_motion_pipeline.example_environment:main",
            "example_task = jaka_motion_pipeline.example_task:main",
            "example_board_sequence = jaka_motion_pipeline.example_board_sequence:main",
            "example_viewpoint_task = jaka_motion_pipeline.example_viewpoint_task:main",
            "export_board_states = jaka_motion_pipeline.export_board_states:main",
        ],
    },
)
