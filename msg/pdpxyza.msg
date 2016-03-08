# PDPXYZA subset of message to be published

std_msgs/Header header
float64 seconds 		# ascii message header bestxyza.amh.seconds
int64   sequence 		# ascii message header bestxyza.amh.sequence 
uint8   p_sol_status            # position solution status
uint8   pos_type 		# position type
float64[3] p                 	# point x y z
float32[3] p_std 		# standard deviation x y z
uint8  v_sol_status 		# velocity solution status
uint8  vel_type			# velocity type
float64[3] v			# velocity x y z
float32[3] v_std                # standard deviation x y z
string stnID			# base station identification - char[4]
float32 v_latency		# latency in the velocity time tag [s]
float32 diff_age		# differential age in [s]
float32 sol_age 		# solution age in [s]
uint8 SVs			# number of satellite vehicles tracked
uint8 solnSVs			# number of satellite vehicles used in solution

