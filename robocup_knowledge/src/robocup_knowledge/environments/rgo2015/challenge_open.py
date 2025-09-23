from robocup_knowledge import knowledge_functions

cabinet = "right_bookcase"
table1  = "dinnertable"
table2  = "bookcase"
table3  = "counter"
object_shelves=["bookcase","bookcase","bookcase"]

objects = ["coke", "meadow_milk", "pringles","oblates","chocosticks","cup"]

#spec find a person and talk with
spec = "(<object1>|<object1> <object2>|<object1> <object2> <object3>)"

choices = {'location':[table1, table2, table3],
'object1':objects,
'object2':objects,
'object3':objects}
