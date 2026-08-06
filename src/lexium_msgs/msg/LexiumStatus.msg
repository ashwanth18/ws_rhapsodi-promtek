# Structured status of the Schneider Lexium Cobot, derived from the
# feedback stream on TCP port 10000 plus driver-side connection state.

std_msgs/Header header

# Driver/connection state
bool connected            # command + feedback sockets are up
bool feedback_fresh       # feedback received within the watchdog timeout

# Control / power state
int32 control_source      # 1: Stick, 2: App, 3: Remote (required for remote motion)
bool powered_on           # arm is energized
bool enabled              # arm is enabled
bool in_position          # arm reached the last commanded target / standstill

# Fault flags (any true blocks/aborts motion)
bool protective_stop
bool emergency_stop
bool collision_stop
bool on_soft_limit

# Last error reported by the controller
string error_code
string error_msg

# Identifier of the currently active move command (echoed by the controller)
int32 command_id
