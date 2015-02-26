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
persons = ["me","a person", "anna", "beth", "carmen", "jennifer", "jessica","kimberly", "kristina", "laura", "mary", "sarah", "alfred", "charles", "daniel", "james", "john", "luis", "paul", "richard", "robert", "steve"]

spec = """ ((<1_action> to (((a|the) <1_locations_rest>)|((an|the) <1_locations_aeuoi>)|(the <1_locations_rooms>))) 
        ((<2_action_person> <2_person>) | (<2_action_count> the <2_objects_types_plural>) | (<2_action> ((the <2_objects_plural>) | (an <2_objects_aeuoi>) | (a <2_objects_rest_singular>))))
            (and (<3_action_special> | (<3_action> (them|it) to <3_location_person>) | (<3_action> to <3_location_person>) | (<3_action_person> to <3_person>))))"""

choices = {"1_action":["go","navigate","move","advance"],
 "1_locations_aeuoi":locations_aeuoi,
 "1_locations_rest":locations_rest,
 "1_locations_rooms":rooms,
 "2_action_person": ["follow","find"],
 "2_person":persons,
 "2_action":["grasp","get","take","find"],
 "2_action_count":["count"],
 "2_objects_plural":objects + object_types_plural,
 "2_objects_types_plural":object_types_plural,
 "2_objects_aeuoi":objects_aeuoi,
 "2_objects_rest_singular":objects_rest + object_types_singular,
 "3_action_special":["tell the time","follow her","follow him"],
 "3_action":["take","move","bring"],
 "3_location_person":locations + rooms + persons,
 "3_action_person":["report"],
 "3_person":persons}
