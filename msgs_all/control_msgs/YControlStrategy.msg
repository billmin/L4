# message of longitudinal control strategy
# throttle
int8 fuzzy_speed_up_type				# fuzzy type of speed-up
bool hold_on_throttle_pedal				# hold the position of throttle pedal
int8 throttle_pedal_traveling_direction	# traveling direction of throttle pedal, i.e. pressed-down or released
float32 throttle_pedal_up_step			# step of releasing throttle pedal
float32 throttle_pedal_down_step		# step of pressing throttle pedal down
# brake
int8 fuzzy_speed_down_type				# fuzzy type of speed-down
bool hold_on_brake_pedal				# hold the position of brake pedal
int8 brake_pedal_traveling_direction	# traveling direction of brake pedal, i.e. pressed-down or released
float32 brake_pedal_up_step				# step of releasing brake pedal
float32 brake_pedal_down_step			# step of pressing brake pedal down
