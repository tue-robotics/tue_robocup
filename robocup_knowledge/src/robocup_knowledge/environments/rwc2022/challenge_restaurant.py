# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

order_grammar = """
O[P] -> ORDER[P] | can i have a ORDER[P] | i would like ORDER[P] | can i get ORDER[P] | could i have ORDER[P] | may i get ORDER[P] | bring me ORDER[P]
ORDER[OO] -> ITEM1[OO] | ITEM2[OO] | ITEM3[OO]
ITEM1[{"food1": F1}] -> ITEM[F1]
ITEM2[{"food1": F1, "food2": F2}] -> ITEM[F1] ITEM[F2] | ITEM[F1] and ITEM[F2]
ITEM3[{"food1": F1, "food2": F2, "food3": F3}] -> ITEM[F1] ITEM[F2] and ITEM[F3] | ITEM[F1] ITEM[F2] ITEM[F3] | ITEM[F1] and ITEM[F2] and ITEM[F3]
"""

# Create grammar
for d in common.objects:
    if d["category"] in ["drink", "fruit", "snack"] or d["name"] == "corn_flakes":
        name = d["name"]
        order_grammar += f"\nITEM['{name}'] -> {name} | a {name} | an {name}"

if __name__ == "__main__":
    import rospy
    from robot_skills.api import Api

    rospy.init_node('blaat')
    a = Api("hero", None)
    rospy.sleep(rospy.Duration(0.5))
    while not rospy.is_shutdown():
        try:
            result = a.query("blaat", order_grammar, "O")
        except:
            pass
        rospy.sleep(rospy.Duration(3.0))
