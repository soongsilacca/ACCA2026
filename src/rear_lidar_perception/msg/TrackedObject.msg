uint32 track_id
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions

# Object ground velocity expressed in header.frame_id.  With valid odometry
# this is ego-motion compensated; otherwise it is relative sensor velocity.
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 relative_velocity

bool is_dynamic
bool odometry_compensated
float32 age
float32 time_since_update

# Each point is prediction_dt farther into the future than the previous one.
# Predictions include constant-velocity object and ego dead reckoning.
float32 prediction_dt
geometry_msgs/Point[] predicted_positions
