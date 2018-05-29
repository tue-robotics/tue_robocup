from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"

grasp_furniture_id1 = "cabinet"
grasp_furniture_id1_area = "on_top_of"

grasp_furniture_id2 = "cabinet"
grasp_furniture_id2_area = "on_top_of"

place_furniture_id = "dinner_table"
place_furniture_id_area = "on_top_of"


options = {
    "orange": {"food": "orange", "drink1": "bifrutas", "drink2": "coke",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"},
    "cereal": {"food": "cereal", "drink1": "fanta", "drink2": "ice_tea",
              "difficult1": "chop_sticks", "difficult2": "spoon", "difficult3": "bowl"}
}
