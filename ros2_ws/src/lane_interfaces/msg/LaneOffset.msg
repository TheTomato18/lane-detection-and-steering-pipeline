# Result of the perception stage: where the lane center sits in the frame.
# The control node combines this with the frame dimensions to compute a
# heading-correction angle, so both are carried in the message.

std_msgs/Header header

# False when no lane pair was detected this frame (lane_center_x is then
# meaningless and should be ignored).
bool detected

# X-pixel of the midpoint between the left and right lane lines, measured
# at the bottom of the frame.
float32 lane_center_x

# Dimensions of the frame the detection was computed on, in pixels.
uint32 frame_width
uint32 frame_height
