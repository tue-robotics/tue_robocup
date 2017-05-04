from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

not_understood_sentences = [
        "I'm so sorry! Can you please speak louder and slower? And wait for the ping!",
        "I am deeply sorry. Please try again, but wait for the ping!",
        "You and I have communication issues. Speak up!",
        "All this noise is messing with my audio. Try again"
    ]

initial_pose = "initial_pose"  # initial pose
starting_pose = "gpsr_meeting_point"  # Designated pose to wait for commands
exit_waypoint = "exit_door_B1"

rooms = common.rooms + ["entrance", "exit"]

translations = { "bookcase" : "bocase" }
