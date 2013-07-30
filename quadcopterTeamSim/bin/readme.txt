readme


INPUT FILES:
waypoints.txt
	gives waypoints, must be in same folder as ./quadcopter_control executable, format:
		each waypoint on its own line, listed as [desx],[desy],[desz],[despsi],[hovertime]
		no commas between fields
drones.txt
	gives the team, must be in same folder as ./quadcopter_control executable, format:
		each drone is its own line, listed as [initx],[inity],[initz],[updateType],[droneType]
		desired heading is assumed to start at -180
		no commas between fields	

formation.txt
	gives the formation of the team, a 2D matrix - diagonal entries will be processed to 0 regardless
	[-1],[l12],[l13],[l14]....
	[l21],[-1],[l23],[l24]...

formationGamma.txt
	gives the formation of the team, a 2D matrix - diagonal entries will be processed to 0 regardless
	[-1],[l12],[l13],[l14]....
	[l21],[-1],[l23],[l24]...	

LOG FILES: 
controlLog logs control signals of real drone only
waypointControlLog logs PID control of waypoint tracking of drone 0, whether it's simulated or not
simlog logs simulated motion of all drones
navlog, edlog, predictlog logs state of real drone only
LFControlLog lots leader-follower control of all drones, simulated or not
