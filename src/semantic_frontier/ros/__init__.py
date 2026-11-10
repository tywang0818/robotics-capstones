"""ROS-facing semantic inference helpers.

The helpers in this package avoid importing ROS modules so they can be tested
with the project-local Python environment. ROS nodes should stay thin wrappers
around these functions.
"""

