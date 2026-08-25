# Dataset meta information of a single object of a perception_msgs/ObjectList

# Lifetime ID of the object, matching perception_msgs/Object.id
uint64 id

# Dataset-specific annotations, e.g. "original_class", "num_lidar_pts" or "attribute".
# Keys may occur more than once, e.g. for objects carrying multiple attributes.
diagnostic_msgs/KeyValue[] info
