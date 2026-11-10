import os
from glob import glob
from setuptools import setup

package_name = "jaka_exploration_core"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"),
         glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "meshes"),
         glob("meshes/*.obj") + glob("meshes/*.stl") + glob("meshes/*.dae")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.rviz") + glob("config/*.srdf")),
        # RViz Web Viewer(FastAPI 服务 + 前端静态页)
        (os.path.join("share", package_name, "rviz_web_viewer"),
         glob("rviz_web_viewer/*.py") + glob("rviz_web_viewer/*.md")
         + glob("rviz_web_viewer/*.txt")),
        (os.path.join("share", package_name, "rviz_web_viewer", "static"),
         glob("rviz_web_viewer/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Exploration core (world, frontier, FSM) for JAKA + D435 sim.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "bootstrap_scan         = jaka_exploration_core.bootstrap_scan_node:main",
            "initial_scan           = jaka_exploration_core.initial_scan_node:main",
            "octomap_to_json_bridge = jaka_exploration_core.octomap_to_json_bridge:main",
            "image_relay            = jaka_exploration_core.image_relay:main",
            "self_filter            = jaka_exploration_core.self_filter:main",
            "frontier_viz           = jaka_exploration_core.frontier_viz:main",
            "periodic_octomap_saver = jaka_exploration_core.periodic_octomap_saver:main",
            "shelf_coverage_monitor = jaka_exploration_core.shelf_coverage_monitor:main",
            "multi_coverage_monitor = jaka_exploration_core.multi_coverage_monitor:main",
            "close_cube_coverage_monitor = jaka_exploration_core.close_cube_coverage_monitor:main",
            "random_frontier_selector = jaka_exploration_core.random_frontier_selector:main",
            "jaka_state_publisher   = jaka_exploration_core.jaka_state_publisher:main",
            "jaka_scan_and_publish  = jaka_exploration_core.jaka_scan_and_publish:main",
            "exploration_logger     = jaka_exploration_core.exploration_logger:main",
            "exploration_report     = jaka_exploration_core.exploration_report:main",
            "viz_scan_diag          = jaka_exploration_core.viz_scan_diag:main",
            "go_home                = jaka_exploration_core.go_home:main",
            "yolo_overlay           = jaka_exploration_core.yolo_overlay_node:main",
            # placeholders for the stages 4-6 nodes
            # "exploration_fsm = jaka_exploration_core.exploration_fsm_node:main",
        ],
    },
)
