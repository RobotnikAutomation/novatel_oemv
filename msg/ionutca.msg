# IONUTCA subset of message to be published

std_msgs/Header header
float64 a0 	# Alpha parameter constant term
float64 a1      # Alpha parameter 1st order term
float64 a2      # Alpha parameter 2nd order term
float64 a3      # Alpha parameter 3rd order term
float64 b0      # Beta parameter constant term
float64 b1      # Beta parameter 1st order term
float64 b2      # Beta parameter 2nd order term
float64 b3      # Beta parameter 3rd order term
uint64  utc_wn	# UTC reference week number
uint64  tot	# Reference time of UTC parameters
float64 A0      # UTC constant term of polynomial
float64 A1      # UTC 1st order term of polynomial
uint64 	wn_lsf  # Future week number
uint64  dn      # Day number (the range is 1 to 7 where
		   # Sunday = 1 and Saturday = 7)
int64 deltat_ls    # Delta time due to leap seconds
int64 deltat_lsf   # Future delta time due to leap seconds
uint64 deltat_utc  # Time difference

