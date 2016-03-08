# BESTPOSA subset of message to be published

std_msgs/Header header
float64 seconds 		# ascii message header bestposa.amh.seconds
int64   sequence 		# ascii message header bestposa.amh.sequence 
uint8   p_sol_status            # position solution status
uint8   pos_type 		# position type
float64 lat                     # latitude
float64 lon                     # longitude
float64 hgt                     # height above mean sea level
float32 lat_std                 # Latitude standard deviation
float32 lon_std                 # Longitude standard deviation
float32 hgt_std                 # Height standard deviation
string stnID			# base station identification - char[4]
float32 diff_age		# differential age in [s]
float32 sol_age 		# solution age in [s]
uint8 SVs			# number of satellite vehicles tracked
uint8 solnSVs			# number of satellite vehicles used in solution

