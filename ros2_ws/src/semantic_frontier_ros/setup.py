from glob import glob

from setuptools import setup


package_name = "semantic_frontier_ros"


setup(
    name=package_name,
    version="0.1.0",
    packages=[
        "semantic_frontier_ros",
        "semantic_frontier",
        "semantic_frontier.segmentation",
        "semantic_frontier.ros",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Boxuan Li",
    maintainer_email="249710153+SULT-630@users.noreply.github.com",
    description="ROS 2 online semantic RGB-D inference for semantic frontier evidence.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "semantic_inference_node = semantic_frontier_ros.semantic_inference_node:main",
            "target_detection_node = semantic_frontier_ros.target_detection_node:main",
        ],
    },
)
