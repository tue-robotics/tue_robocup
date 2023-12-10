result1 = {
  'intent': "demo-presentation",
  'slots' : {
    'language': "dutch"
  }
}

result2 = {
  "intent": "place",
  "slots": {
    "object": "ice tea",
    "source-location": "work bench one",
    "target-location": "dinner table",
  }
}

result3 = {
  'intent': "say",
  'slots': {
    'sentence': "me what day it is"
  }
}

result4 = {
  "intent": "find",
  "slots": {
    "object": "waiving person"
  }
}

say_dict = {
    "your name": "ROBOT_NAME",
    "the time": "TIME",
    "what time it is": "TIME",
    "the name of your team": "my team is tech united",
    "the day of the month": "DAY_OF_MONTH",
    "the day of the week": "DAY_OF_WEEK",
    "what day is today": "TODAY",
    "me what day it is": "TODAY",
    "the date": "TODAY",
    "a joke": "joke"
}


def create_semantics(result):
    action = result.get("intent", {})
    slots_dict = result.get("slots", {})

    semantics = {
        "action": action
    }
    # ToDO: deal with waiving person
    # ToDo: implement learn person
    if action in ["inspect", "place", "hand-over", "navigate-to", "find"]:
        for key, value in slots_dict.items():
            value = value.replace(" one", "1")
            value = value.replace(" two", "2")
            value = value.replace(" ", "_")
            if key.endswith("location") or key == "entity":
                semantics[key] = {"id": value}
            if key == "object":
                semantics[key] = {"type": value} if not value == "someone" else {"type": "person"}
                print(value)
                if value == "waiving_person":
                    print(value)
                    semantics[key] = {"type": "person", "tags": ["LWave", "Rwave"]}
            if value == "me":
                semantics[key] = {"id": "operator"}

    elif action == "demo-presentation":
        for key, value in slots_dict.items():
            if key == "language":
                semantics[key] = "nl" if value == "dutch" else "en"

    elif action == "say":
        for key, value in slots_dict.items():
            if value in say_dict:
                semantics[key] = say_dict[value]

    return semantics


print(create_semantics(result4))