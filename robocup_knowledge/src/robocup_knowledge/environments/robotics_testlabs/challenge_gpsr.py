from robocup_knowledge import knowledge_loader
challenge_speech_recognition_data = knowledge_loader.load_knowledge("challenge_speech_recognition")

spec_questions = challenge_speech_recognition_data.spec 
choices_questions = challenge_speech_recognition_data.choices 

# rooms
rooms = ["kitchen", "bedroom", "living_room"]

# mapping from furniture to small objects that are on top of them (and can be grabbed)
furniture_to_objects = { "cabinet": ["beer", "bifrutas", "coffee_pads", "coke", "deodorant"],
                         "dinnertable": ["fanta", "ice_tea", "mentos", "sprite", "tea", "teddy_bear", "water", "xylit24_spearmint", "xylit24_white"] }