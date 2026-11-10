from glob import glob

from setuptools import find_packages, setup


package_name = "mp_team_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/examples", glob("../../examples/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Motion Planning Integration Team",
    maintainer_email="student@example.com",
    description="Team adapters for the integrated JAKA A5 planning workspace.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "environment_bridge = mp_team_bridge.environment_bridge:main",
            "decision_bridge = mp_team_bridge.decision_bridge:main",
            "integration_test_client = mp_team_bridge.integration_test_client:main",
            "motion_metrics_reporter = mp_team_bridge.motion_metrics_reporter:main",
            "publish_json_file = mp_team_bridge.publish_json_file:main",
        ],
    },
)
