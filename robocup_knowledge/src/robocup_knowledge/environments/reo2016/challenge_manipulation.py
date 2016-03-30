''' Cabinet used for the challenge '''
cabinet = "bookcase"

''' Shelves where objects might be '''
object_shelves =["shelf5", "shelf4", "shelf3", "shelf1"]

''' Shelf where we will actually try to grasp '''
grasp_shelf = cabinet + "/shelf6"

''' Shelf where we will actually place stuff '''
place_shelf = cabinet + "/shelf5"

''' Room where everything will take place '''
room = "livingroom"

'''Object types that can be recognized'''
object_types = [o["name"] for o in common.objects if o["category"] != "container"]
