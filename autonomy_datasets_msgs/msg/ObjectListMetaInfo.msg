# Dataset meta information for the objects of a perception_msgs/ObjectList
#
# perception_msgs/Object has no field for dataset-specific annotations, so they are published
# on a separate "<object list topic>/meta_info" topic next to the object list they belong to.
# Consumers correlate both topics via the header stamp and the object id.

# Header of the perception_msgs/ObjectList this meta information belongs to
std_msgs/Header header

# ID of the dataset scene/sequence/clip the object list was taken from
string scene_id

# Meta information per object; objects without any meta information are omitted
ObjectMetaInfo[] objects
