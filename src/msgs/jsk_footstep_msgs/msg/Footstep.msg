uint8 RIGHT=2
uint8 LEFT=1

# Constants to visualize progress
uint8 REJECTED=3
uint8 APPROVED=4

## limb_indicator values
uint8 LLEG=1
uint8 RLEG=2
uint8 LARM=5
uint8 RARM=6

uint8 leg ## value should be one of limb_indicator values.
geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.
duration duration
# optional parameters
uint32 footstep_group
geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]
geometry_msgs/Vector3 offset     ## offset from pose to center of cube
float32 swing_height
float32 cost
