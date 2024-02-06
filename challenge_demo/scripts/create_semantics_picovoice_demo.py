say_dict = {
    "your name": "ROBOT_NAME",
    "the time": "TIME",
    "what time it is": "TIME",
    "the name of your team": "TEAM_NAME",
    "the day of the month": "DAY_OF_MONTH",
    "the day of the week": "DAY_OF_WEEK",
    "what day is today": "TODAY",
    "me what day it is": "TODAY",
    "the date": "TODAY",
    "a joke": "joke"
}


def create_semantics(result):

    actions_dict = {
        "actions": []
    }

    if result["action"] in ["inspect", "place", "hand-over", "navigate-to", "find"]:
        for key, value in result.items():
            value = value.replace(" one", "1")
            value = value.replace(" two", "2")
            value = value.replace(" ", "_")
            if key.endswith("location") or key == "entity":
                result[key] = {"id": value}
            if key == "object":
                result[key] = {"type": value} if not value == "someone" else {"type": "person"}
                if value == "waiving_person":
                    result[key] = {"type": "person", "tags": ["LWave", "Rwave"]}
            if value == "me":
                result[key] = {"id": "operator"}

    elif result["action"] == "demo-presentation":
        result["language"] = "nl" if result["language"] == "dutch" else "en"

    elif result["action"] == "say":
        for key, value in result.items():
            if value in say_dict:
                result[key] = say_dict[value]

    actions_dict["actions"].append(result)

    return actions_dict


action_handlers = {
    "inspect": lambda result: {"action": "inspect", "entity": {"id": result.get("entity", None)}},

    "place": lambda result: {
        "action": "place",
        "source-location": {"id": result.get("source-location", None).replace(" one", "1").replace(" two", "2")},
        "target-location": {"id": result.get("target-location", None).replace(" one", "1").replace(" two", "2")},
        "object": {"type": result.get("object", None).replace(" ", "_")}
    },

    "hand-over": lambda result: {
        "action": "hand-over",
        "source-location": {"id": result.get("source-location", None).replace(" one", "1").replace(" two", "2")},
        "target-location": {"id": "operator"},
        "object": {"type": result.get("object", None)}
    },

    "navigate-to": lambda result: {"action": "navigate-to",
                                   "target-location": {"id": result.get("target-location", None)}},

    "find": lambda result: {"action": "find", "object": {
        "type": result.get("object", None) if not result["object"] == "someone" else "person", "tags": result.get("tags", None)},
                            "location": {"id": result.get("source-location", None)}},

    "demo-presentation": lambda result: {"action": "demo-presentation",
                                         "language": "nl" if result["language"] == "dutch" else "en"},

    "say": lambda result: {"action": "say", "sentence": say_dict.get(result["sentence"], result["sentence"])}
}


def create_semantics_generic(result):
    action = result["action"]

    if action in action_handlers:
        action_dict = action_handlers[action](result)
        return {"actions": [action_dict]}
    else:
        return {"actions": []}
