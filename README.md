QuadcopterMapping
=================

Mapping large objects using multiple autonomous ARDrones.

Notes:
	To kill ghost ros nodes and processes: 
		rosnode list: to get a list of all running nodes
		rosnode kill [nameOfNode]
		rosnode cleanup: if kill didn't kill it

void ardrone_at_set_progress_cmd( int32_t enable, float32_t phi, float32_t theta, float32_t gaz, float32_t yaw )
keep values between -1.0 and 1.0

