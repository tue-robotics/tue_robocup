# RESTAURANT KNOWLEDGE FILE RWC2018

# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

order_grammar = """
O[P] -> ORDER[P] | can i have a ORDER[P] | i would like ORDER[P] | can i get ORDER[P] | could i have ORDER[P] | may i get ORDER[P] | bring me ORDER[P]
ORDER[<A1>] -> ITEMS[A1] | DET ITEMS[A1]
ORDER[<A1, A2>] -> ITEMS[A1] ITEMS[A2] | DET ITEMS[A1] DET ITEMS[A2] | DET ITEMS[A1] and DET ITEMS[A2] | ITEMS[A1] and ITEMS[A2]
ORDER[<A1, A2, A3>] -> ITEMS[A1] ITEMS[A2] ITEMS[A3] | ITEMS[A1] ITEMS[A2] and ITEMS[A3] |DET ITEMS[A1] DET ITEMS[A2] DET ITEMS[A3] | DET ITEMS[A1] and DET ITEMS[A2] and DET ITEMS[A3] | DET ITEMS[A1] DET ITEMS[A2] and DET ITEMS[A3]
DET -> a | an
"""

# ORDER[OO] -> COMBO[OO] | BEVERAGE[OO]
# BEVERAGE[{"beverage": B}] -> BEV[B]
# BEVERAGE[{"beverage": B}] -> DET BEV[B]
# COMBO[{"food1": F1, "food2": F2}] -> FOOD[F1] and FOOD[F2] | FOOD[F1] with FOOD[F2]
# COMBO[{"food1": F1, "food2": F2}] -> DET FOOD[F1] and FOOD[F2] | DET FOOD[F1] with FOOD[F2]
# COMBO[{"food1": F1, "food2": F2}] -> FOOD[F1] and DET FOOD[F2] | FOOD[F1] with DET FOOD[F2]
# COMBO[{"food1": F1, "food2": F2}] -> DET FOOD[F1] and DET FOOD[F2] | DET FOOD[F1] with DET FOOD[F2]

# COMBO[{"food1": F1, "food2": F2}] -> DET FOOD[F1] and DET FOOD[F2] | DET FOOD[F1] with DET FOOD[F2]
# BEVERAGE[{"beverage": B}] -> DET BEV[B]

candies = ["biscuit", "frosty_fruits", "snakes"]
drinks = ["beer", "chocolate_milk", "coke", "juice", "lemonade", "water"] #"tea bag"
food = ["carrot", "cereals", "noodles", "onion", "vegemite"]
fruits = ["apple", "kiwi", "lemon", "orange", "pear"]
snacks = ["cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties"]

# Add drinks
# for d in common.objects:
    # if d["category"] == "drink":
    #     order_grammar += "\nBEV['{}'] -> {}[B]".format(d["name"], d["name"].replace('_', ' '))
    # elif d["category"] == "food" or d["category"] == "snack":
    #     order_grammar += "\nFOOD['{}'] -> {}".format(d["name"], d["name"].replace('_', ' '))

for item in candies + drinks + food + fruits + snacks:
    order_grammar += "\nITEMS['{}'] -> {}[B]".format(item, item.replace('_', ' '))

if __name__ == "__main__":
    import rospy
    from robot_skills.api import Api
    rospy.init_node('blaat')
    a = Api("", None)
    rospy.sleep(rospy.Duration(0.5))
    while True:
        result = a.query("blaat", order_grammar, "O")
        import ipdb;ipdb.set_trace()
