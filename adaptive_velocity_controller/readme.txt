Adaptive Velocity Controller Node

This node calculates the servo velocity based and increases/decrease it adaptively according to three parameters which are:
	- Local plan curvature (in the range [0,1])
	- Global plan curvature (in the range [0,1])
	- Clear path ratio (in the range [0,1])

The node is dynamically reconfigurable using rqt_reconfigure. It has the following dynamic parameters:
 * MAX_LOOK_AHEAD_DIST: Clear path calculations are done up to this much of distance in the virtual lane
 * MIN_LOOK_AHEAD_DIST: Clear path calculations are done from this much of distance on in the virtual lane
 * DISTANCE_TOLERANCE: The tolerance of the scan ranges in the virtual lane. Measurements within this tolerance
 *                      are considered as no obstacle in the virtual lane.
 * ignore_local_planner_vel_cmd: Boolean variable for ignoring/considering the cmd_vel messages from move_base
 * MAX_SPEED: Maximum servo velocity
 * MIN_SPEED: Minimum servo velocity
 * EXP_L: Local plan curvature weighting exponential in velocity calculation.
 * EXP_G: Global plan curvature weighting exponential in velocity calculation.
 * EXP_C: Clear path ratio plan weighting exponential in velocity calculation.

The node first creates a lookup table to compare it with laser scans in real time, in order to compute the distance to the nearest obstacle.
It subscribes to local_plan, global_plan and cmd_vel topics. In each local_plan callback it calculates the local plan curvature and in each
global plan callback it calculates the global plan curvature. Using the cmd_vel, it maps the cmd_vel message from move_base to servo command range. The range of servo command is between MIN_SPEED and MAX_SPEED parameters. (The function can ignore the velocity commands from move base, if set by user)

Depending on
	1. Local plan curvature,
	2. Global plan curvature and
	3. Ratio of minimum obstacle distance to MAX_LOOK_AHEAD_DIST (defined as clear path ratio),
the velocity is adaptively changed. The more linear the global and local plans are and the more is the distance to the nearest obstacle (minimum_obstacle_distance), the higher becomes the servo velocity command. The weighting factor of each component (local plan curvature, global plan curvature and ratio of minimum obstacle distance to MAX_LOOK_AHEAD_DIST) is calculated by taking the EXP_L, EXP_G and EXP_C power of each factor. Each factor is a number between zero and one.

Finally, the servo velocity is sent as an Int16 message to tas_autonomous_control_node, which then publishes the servo commands.
