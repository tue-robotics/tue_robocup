from robocup_knowledge import knowledge_functions

initial_pose = "initial_pose_door_c"  # initial pose
starting_pose = "gpsr_meeting_point"  # Designated pose to wait for commands
exit_waypoint = "exit_door_B1"

rooms = knowledge_functions.rooms + ["entrance", "exit", "office"]

translations = { "bookcase" : "bocase" }
