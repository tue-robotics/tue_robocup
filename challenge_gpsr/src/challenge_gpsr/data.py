objects_aeuoi = ["orange drink", "energy drink"]
objects_rest = ["coke", "fanta", "milk", "beer", "seven up", "coffee", "tea", "frutas"]
objects = objects_aeuoi + objects_rest

object_types_plural = ["drinks","snacks","cleaning stuff", "food"]
object_types_singular = ["drink","snack"]
object_types = object_types_plural + object_types_singular

locations_aeuoi = ["appliance"]
locations_rest = ["bed", "shelf", "table", "trash bin", "bin", "workbench","desk","bar","tulip","plant","dinner table","couch table", "kitchen counter","nightstand"]
locations = locations_aeuoi + locations_rest
rooms = ["kitchen", "hallway", "living room", "bedroom", "corridor", "workshop"]
persons_men = ["anna", "beth", "carmen", "jennifer", "jessica","kimberly", "kristina", "laura", "mary", "sarah"]
persons_women = ["alfred", "charles", "daniel", "james", "john", "luis", "paul", "richard", "robert", "steve"]
persons_general = ["me","a person"] 
persons = persons_general + persons_men + persons_women

spec123_1 = "(<1_action> to the <1_locations_rooms> <2_action_person> ((<2_person_men> and (<3_action_follow_men>|<3_action_interact>))|(<2_person_women> and (<3_action_follow_women>|<3_action_interact>))|(<2_person_me> and (<3_action_follow_me>|<3_action_interact>))|(<2_person_general> and <3_action_interact>)))"

spec1 = "(<1_action> to (((a|the) <1_locations_rest>)|((an|the) <1_locations_aeuoi>)|(the <1_locations_rooms>)) )"
spec23_1 = "((<2_action> ((the <2_objects_plural>)|(an <2_objects_aeuoi>)|(a <2_objects_rest_singular>))) (and <3_action> (them|it) to ((the <3_location>)|<3_person_me>)))"
spec23_2 = "(<2_action_count> the <2_objects_types_plural> and <3_action_report> to me)"

spec = "("+spec123_1+"|("+spec1+"("+spec23_1+"|"+spec23_2+")))"

choices = {"1_action":["go","navigate","move","advance"],	
 "1_locations_aeuoi":locations_aeuoi,
 "1_locations_rest":locations_rest,
 "1_locations_rooms":rooms,
 "2_action_person": ["find"],
 "2_person_men":persons_men,
 "2_person_women":persons_women,
 "2_person_me":["me"],
 "2_person_general":["a person"],
 "2_action":["grasp","get","take","find"],
 "2_action_count":["count"],
 "2_objects_plural":objects + object_types_plural,
 "2_objects_types_plural":object_types_plural,
 "2_objects_aeuoi":objects_aeuoi,
 "2_objects_rest_singular":objects_rest + object_types_singular,
 "3_action_follow_men":["follow him"],
 "3_action_follow_women":["follow her"],
 "3_action_follow_me":["follow me"],
 "3_action_interact":["tell the time","answer the question", "answer a question"],
 "3_action":["take","move","bring"],
 "3_location":locations + rooms,
 "3_action_report":["report",],
 "3_person_me":["me"]}
