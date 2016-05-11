# Informations about a behavior defining a vector field

string behavior_id  
string type			# the type of vector field (waypoint, patrol_circle, limite, line)
float64 xa			# x position of the waypoint and the center of the circle OR x position of one limit of the segment
float64 ya			# y position of the waypoint and the center of the circle OR y position of one limit of the segment
float64 xb			# x position of the other limit of the segment
float64 yb			# y position of the other limit of the segment
float64 r			# length of the attraction of the behavior
float64 s 			# attractive force of the behavior