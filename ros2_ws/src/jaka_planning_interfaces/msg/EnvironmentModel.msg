std_msgs/Header header
# Strictly increasing immutable snapshot identifier.
uint64 version

# When true, the incoming OctoMap replaces the current planning-scene OctoMap.
bool replace
octomap_msgs/OctomapWithPose octomap
