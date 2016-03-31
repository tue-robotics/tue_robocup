from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

''' Cabinet used for the challenge '''
cabinet = "bookcase"

''' Shelves where objects might be '''
object_shelves =["shelf6", "shelf5", "shelf4", "shelf3", "shelf1"]

''' Shelf where we will actually try to grasp '''
grasp_shelf = "shelf3"

''' Shelf where we will actually place stuff '''
place_shelf = "shelf2"

''' Room where everything will take place '''
room = "livingroom"

'''Object types that can be recognized'''
object_types = [o["name"] for o in common.objects if o["category"] != "container"]

# ToDo: make nice
min_grasp_height = 0.73
max_grasp_height = 1.02
