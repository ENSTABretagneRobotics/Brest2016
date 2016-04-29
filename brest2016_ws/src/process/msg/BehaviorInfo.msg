# Informations about a behavior defining a vector field

string behavior_type    # the type of vector field (attractive point, line...)
float64 position_x
float64 position_y
float64 theta			# rotation of the vector field. Useless for some fields
float64 scale
float64 param1			# radius for circle vector field OR direction for rotation field
float64 param2			# rho parameter for circle field