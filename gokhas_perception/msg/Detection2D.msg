# Defines a 2D detection result.
#
# This is similar to a 2D classification, but includes position information,
#   allowing a classification result for a specific crop or image point to
#   to be located in the larger image.

std_msgs/Header header
int8 track_id
# Class probabilities
vision_msgs/ObjectHypothesisWithPose[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# The 2D data that generated these results (i.e. region proposal cropped out of
#   the image). Not required for all use cases, so it may be empty.
sensor_msgs/Image source_img