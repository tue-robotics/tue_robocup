from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

# Do not alter the keys please :)
guiding_spec = "(<location> <side>|Please follow me)"

# Do not alter keys or side values
guiding_choices = {"location" : ["one", "two", "three"], "side" : ["left", "right", "front"]}

navigation_position_constraint_operator = "x^2 + y^2 < 0.2^2"

kitchen_radius = 2

# Do not alter the keys please :)
order_spec = "(<beverage>|<food1> and <food2>)"

# Do not alter keys or side values
drinks = [ o["name"] for o in common.objects if o["category"] == "drink" ] 
foods = [ o["name"] for o in common.objects if o["category"] == "food" ] 
order_choices = {"beverage": drinks, "food1": foods, "food2": foods }
