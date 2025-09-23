from robocup_knowledge import knowledge_functions
initial_pose_sergio = "initial_pose_door_B"

initial_pose_amigo = "initial_pose_door_B"

# first map 3 locations + objects on it
explore_location_1 = "final_cupboard"
explore_location_2 = "final_dinnertable"
explore_location_3 = "final_desk"
explore_locations_part_1 = [explore_location_1, explore_location_2, explore_location_3]

# then the location where sergio should receive the command of the operator (at the jury)
task_location_sergio = "task_location_sergio"

# then continue mapping
explore_location_4 = "final_tv"
explore_location_5 = "final_kitchencounter"
explore_location_6 = "final_cabinet"
explore_location_7 = "final_bartable"
explore_location_8 = "final_small_table_1"
explore_location_9 = "final_small_table_2"
explore_location_10 = "final_coathanger"
explore_location_11 = "final_left_bedside_table"
explore_location_12 = "final_right_bedside_table"
explore_locations_part_2 = [explore_location_4, explore_location_5, explore_location_6, explore_location_7, explore_location_8, explore_location_9, explore_location_10, explore_location_11, explore_location_12]

# last position where sergio will wait (between cabinet and bartable, near the wall)
end_location_sergio = "final_last_position_sergio"

# last position where sergio will wait (between cabinet and bartable, near the wall)
end_location_amigo = "final_last_position_amigo"

#SPECS + CHOICES for speech recognition
location_options = ['cup_board','dinner_table','desk','side_table','tv','kitchen_counter','cabinet','bartable','couch','chair','coat_hanger','left_bedside_table','right_bedside_table']
object_options = ['coke','pringles','choco_sticks','beer','juice','tea','coffee']

#mesh_spec = "((it is (a|an) <object>)|(a <object>)|(an <object>)|<object>)"
mesh_spec = "(<object>)"
mesh_choices = {'object':location_options}

#object_spec = "((it is a <object>)|(a <object>)|(an <object>))"
object_spec = "(<object>)"
object_choices = {'object':object_options}

operator_object_spec = "((Can you give me a <object>)|<object>)"
operator_object_choices = {'object':object_options}

