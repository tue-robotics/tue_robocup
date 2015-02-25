objects_aeuoi = ["orange juice"]
objects_rest = ["beer", "milk", "sevenup"]
objects = objects_aeuoi + objects_rest
object_types_plural = ["drinks","snacks","cleaningstuff", "food"]
object_types_singular = ["drink","snack"]
object_types = object_types_plural + object_types_singular

locations_aeuoi = ["appliance"]
locations_rest = ["bed", "shelf", "table", "trashbin", "bin"]
locations = locations_aeuoi + locations_rest
rooms = ["kitchen", "hallway", "livingroom", "bedroom", "corridor"]
persons = ["me","a person", "anna", "beth", "carmen", "jennifer", "jessica","kimberly", "kristina", "laura", "mary", "sarah", "alfred", "charles", "daniel", "james", "john", "luis", "paul", "richard", "robert", "steve"]

spec = """
(<question_1>|<question_2>|<question_3>|<question_4>|<question_5>|<question_6>|<question_7>|<question_8>|<question_9>|<question_10>|
<question_11>|<question_12>|<question_13>|<question_14>|<question_15>|<question_16>|<question_17>|<question_18>|<question_19>|<question_20>|
<question_21>|<question_22>|<question_23>|<question_24>|<question_25>)"""

choices = {"question_1":["what time is it"],
"question_2":["what is the capital of germany"],
"question_3":["what is the heaviest animal in the world"],
"question_4":["who is the president of america"],
"question_5":["what is your name"],
"question_6":["who is your example"],
"question_7":["what is your motto"],
"question_8":["which football club is the best"],
"question_9":["who is the best looking person around here"],
"question_10":["which town has been bombed"],
"question_11":["which person is not able to say yes"],
"question_12":["what is the capital of brazil"],
"question_13":["what is the oldest drug used on earth"],
"question_14":["in which year was robocup founded"],
"question_15":["how many rings has the olympic flag"],
"question_16":["what is the worlds most popular green vegetable"],
"question_17":["which insect has the best eyesight"],
"question_18":["who lives in a pineapple under the sea"],
"question_19":["what is your teams name"],
"question_20":["what is the answer to the ultimate question about life the universe and everything"],
"question_21":["what is the capital of poland"],
"question_22":["which country grows the most potatoes"],
"question_23":["which country grew the first orange"],
"question_24":["how many countries are in europe"],
"question_25":["which fish can hold objects in its tail"]}

answers = {"question_1":["time to buy a watch"],
"question_2":["berlin"],
"question_3":["Is quite ok"],
"question_4":["barack obama"],
"question_5":["amigo"],
"question_6":["erik geerts"],
"question_7":["yolo"],
"question_8":["feyenoord"],
"question_9":["erik geerts of course"],
"question_10":["schijndel"],
"question_11":["dirk holtz"],
"question_12":["brasilia"],
"question_13":["alcohol"],
"question_14":["nineteen ninety seven"],
"question_15":["five"],
"question_16":["lettuce"],
"question_17":["dragonfly"],
"question_18":["spongebob squarepants"],
"question_19":["tech united eindhoven"],
"question_20":["forty two"],
"question_21":["warsaw"],
"question_22":["russia"],
"question_23":["china"],
"question_24":["fifty"],
"question_25":["sea horse"]}

###############
## TEST DATA ##
###############
## Questions

# choices = {"question_1":"what time is it"],
# "question_2":["what is the capital of germany"],
# "question_3":["what is the heaviest animal in the world"],
# "question_4":["who is the president of america"],
# "question_5":["what is your name"],
# "question_6":["who is your example"],
# "question_7":["what is your motto"],
# "question_8":["which football club is the best"],
# "question_9":["who is the best looking person around here"],
# "question_10":["which town has been bombed"],
# "question_11":["which person is not able to say yes"],
# "question_12":["what is the capital of brazil"],
# "question_13":["what is the oldest drug used on earth"],
# "question_14":["in which year was robocup founded"],
# "question_15":["how many rings has the olympic flag"],
# "question_16":["what is the worlds most popular green vegetable"],
# "question_17":["which insect has the best eyesight"],
# "question_18":["who lives in a pineapple under the sea"],
# "question_19":["what is your teams name"],
# "question_20":["what is the answer to the ultimate question about life the universe and everything"],
# "question_21":["what is the capital of poland"],
# "question_22":["which country grows the most potatoes"],
# "question_23":["which country grew the first orange"],
# "question_24":["how many countries are in europe"],
# "question_25":["which fish can hold objects in its tail"]}

## Answers:

# answers = {"question_1":["time to buy a watch"],
# "question_2":["berlin"],
# "question_3":["Is quite ok"],
# "question_4":["barack obama"],
# "question_5":["amigo"],
# "question_6":["erik geerts"],
# "question_7":["yolo"],
# "question_8":["feyenoord"],
# "question_9":["erik geerts of course"],
# "question_10":["schijndel"],
# "question_11":["dirk holtz"],
# "question_12":["brasilia"],
# "question_13":["alcohol"],
# "question_14":["nineteen ninety seven"],
# "question_15":["five"],
# "question_16":["lettuce"],
# "question_17":["dragonfly"],
# "question_18":["spongebob squarepants"],
# "question_19":["tech united eindhoven"],
# "question_20":["forty two"],
# "question_21":["warsaw"],
# "question_22":["russia"],
# "question_23":["china"],
# "question_24":["fifty"],
# "question_25":["sea horse"]}

