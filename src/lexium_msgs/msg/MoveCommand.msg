# A single Lexium move command in protocol-native units (mm and degrees).
# Sent verbatim to the controller, so values are NOT remapped by the driver's
# joint_sign / joint_offset_deg calibration.

# Which move primitive to use.
string MOVE_J=moveJ        # joint move; target = [j1..j6] in degrees
string MOVE_L=moveL        # linear move; target = [x,y,z,rx,ry,rz] (mm, deg)
string MOVE_TCP=moveTCP    # cartesian joint move; target = [x,y,z,rx,ry,rz]
string MOVE_C=moveC        # circular move; target = end pose, circ = via pose

string move_type           # one of the constants above (default moveJ if empty)
float64[] target           # joint angles (deg) or cartesian pose (mm/deg)
float64[] circ             # moveC only: circular/via pose [x,y,z,rx,ry,rz]

float64 speed              # deg/s (moveJ) or mm/s (cartesian); <= 0 => driver default
float64 acc                # deg/s^2 or mm/s^2; <= 0 => driver default
float64 arc_transition     # blend radius; 0 => stop at this point (clean speed change)
int32 rel_flag             # 0 = absolute (default)
