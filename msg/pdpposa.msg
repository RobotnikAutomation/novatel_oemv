# PDPPOSA subset of message to be published

std_msgs/Header header
float64 seconds 		# ascii message header pdpposa.amh.seconds
int64   sequence 		# ascii message header pdpposa.amh.sequence 
uint8   p_sol_status    # position solution status
uint8   pos_type 		# position type
float64 lat             # latitude
float64 lon             # longitude
float64 hgt             # height above mean sea level
float32 undulation		# Undulation - the relationship between the geoid and the WGS84 ellipsoid (m^2)
int32   datum_id		# Datum ID number
float32 lat_std         # Latitude standard deviation
float32 lon_std         # Longitude standard deviation
float32 hgt_std         # Height standard deviation
string  stnID			# base station identification - char[4]
float32 diff_age		# differential age in [s]
float32 sol_age 		# solution age in [s]
uint8 SVs			    # number of satellite vehicles tracked
uint8 solnSVs			# number of satellite vehicles used in solution
uint8 ext_sol_stat      # (hex) extended solution status
uint8 galileo_beidou_sig_mask  # (hex) signals used mask 0=unknown   
uint8 gps_glonass_sig_mask     # (hex) signals used mask 0=unknown   


