# One planning cycle of board_pursuit_planner, for bags and grading.
#
# v_controller is what the student CruiseController returned, before the
# shell's clamps; v_command is what went into the trajectory. limit names the
# clamp that bound, or is empty when none did.

std_msgs/Header header      # stamp = planning time; frame_id = trajectory frame
bool    valid               # the CoachState the cycle used was valid and fresh
bool    autonomous          # operation mode was AUTONOMOUS this cycle
float32 range               # m, as received in CoachState
float32 range_rate          # m/s, as received
float32 bearing             # rad, as received
float32 ego_speed           # m/s, from /localization/kinematic_state
float32 v_controller        # m/s, CruiseController.update() output (NaN if it raised)
float32 v_command           # m/s, after clamps; the speed written into the trajectory
float32 s_stop              # m along the path where velocity becomes zero
string  limit               # clamp that bound: v_max, accel, decel, standoff, invalid, error, or empty
