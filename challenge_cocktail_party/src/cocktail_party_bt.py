#! /usr/bin/env python

# pi_trees package
from pi_trees_lib.pi_trees_lib import *

# other imports
import time


################################# SETUP VARIABLES ######################################


MIN_SIMULTANEOUS_ORDERS = 2   # Amigo won't fetch the drinks until he has this amount of requests
MAX_SIMULTANEOUS_ORDERS = 3   # Amigo will imediatly fetch the drinks when he has reached this amount of requests
MAX_DRINKS_SERVED       = 3   # The challenge will finish when Amigo has served this amount of requests


#########################################################################################



class CocktailPartyBT():
    def __init__(self):
        # The root node
        ROOT_NODE = Sequence("ROOT_NODE")
        

        ############## Get request from people ##############

        NAV_LIVING_ROOM_t = NavLivingRoom("NAV_LIVING_ROOM_t")
        GET_PERSON_REQ_t = GetPersonRequest("GET_PERSON_REQ_t")
        CHECK_REQ_t = CheckPendingRequests("CHECK_REQ_t", 3)
        TAKE_REQUEST_sl = Selector("TAKE_REQUEST_sl")
        GET_REQUESTS_sq = Sequence("GET_REQUESTS_sq")


        TAKE_REQUEST_sl.add_child(CHECK_REQ_t)
        TAKE_REQUEST_sl.add_child(GET_PERSON_REQ_t)
        
        GET_REQUESTS_sq.add_child(NAV_LIVING_ROOM_t)
        GET_REQUESTS_sq.add_child(TAKE_REQUEST_sl)


        ############## Get drinks requested ##############

        NAV_KITCHEN_t = NavKitchen("NAV_KITCHEN_t")
        GET_DRINKS_sq = Sequence("GET_DRINKS_sq")
        FETCH_DRINK_t = FetchDrinks("FETCH_DRINK_t")

        GET_DRINKS_sq.add_child(NAV_KITCHEN_t)
        GET_DRINKS_sq.add_child(FETCH_DRINK_t)



        ############## Serve drinks to people ##############

        SERVE_DRINKS = Sequence("SERVE_DRINKS")


        COCKTAIL_PARTY = Sequence("COCKTAIL_PARTY")
        COCKTAIL_PARTY.add_child(GET_REQUESTS_sq)
        COCKTAIL_PARTY.add_child(GET_DRINKS_sq)

        ROOT_NODE.add_child(COCKTAIL_PARTY)

        
        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(ROOT_NODE)

        print "\n\n Starting ROOT_NODE"
            
        # Run the tree
        while True:
            status = ROOT_NODE.run()
            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break


#########################################################################
#########################################################################

class FetchDrinks(Task):
    def __init__(self, name, *args, **kwargs):
        super(FetchDrinks, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.count = 0
        print "[", self.name, "]", "Creating task"
 
    def run(self):
        print "[", self.name, "]", "Run"
        time.sleep(0.5)

        print "Fetching a drink"
        
        print "[", self.name, "]", "Return SUCCESS\n"
        return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]", "Reset"


#########################################################################

class CheckPendingRequests(Task):
    def __init__(self, name, requestsN, *args, **kwargs):
        super(CheckPendingRequests, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.count = 0
        self.requestsN = requestsN

        print "[", self.name, "]", "Creating task"
 
    def run(self):
        print "[", self.name, "]", "Run"
        time.sleep(0.5)

        self.count = self.count + 1

        if self.count < self.requestsN:
            print "Number of requests is not enough:", self.count, self.requestsN
            print "[", self.name, "]", "Return FAILURE\n"
            return TaskStatus.FAILURE
        else:
            print "[", self.name, "]", "Return SUCCESS\n"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]", "Reset"

#########################################################################

class NavKitchen(Task):
    def __init__(self, name, *args, **kwargs):
        super(NavKitchen, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.count = 0
        print "[", self.name, "]", "Creating task"
 
    def run(self):
        print "[", self.name, "]", "Run"
        time.sleep(0.5)

        self.count = self.count + 1
        print "[", self.name, "]", "Navigating to the kitchen..."

        # has the robot arrived at the kitchen?
        if self.count < 2:
            print "[", self.name, "]", "Return RUNNING"
            return TaskStatus.RUNNING
        else:
            self.count = 0
            print "[", self.name, "]", "Return SUCCESS\n"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]", "Reset"

#########################################################################

class NavLivingRoom(Task):
    def __init__(self, name, *args, **kwargs):
        super(NavLivingRoom, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.count = 0
        print "[", self.name, "]", "Creating task"
 
    def run(self):
        print "[", self.name, "]", "Run"
        time.sleep(0.5)

        self.count = self.count + 1
        print "[", self.name, "]", "Navigating to the living room..."

        # has the robot arrived at the living room?
        if self.count < 2:
            print "[", self.name, "]", "Return RUNNING"
            return TaskStatus.RUNNING
        else:
            self.count = 0
            print "[", self.name, "]", "Return SUCCESS\n"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]", "Reset"

#########################################################################

class GetPersonRequest(Task):
    def __init__(self, name, *args, **kwargs):
        super(GetPersonRequest, self).__init__(name, *args, **kwargs)
        
        self.name = name
        print "[", self.name, "]", "Creating task"
 
    def run(self):
        print "[", self.name, "]", "Run"
        time.sleep(0.5)

        print "[", self.name, "]", "John Doe asked for a coke"

        print "[", self.name, "]", "Return SUCCESS\n"
        return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]", "Reset"


#########################################################################


if __name__ == '__main__':
    tree = CocktailPartyBT()