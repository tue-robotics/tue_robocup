#! /usr/bin/env python

# pi_trees package
from pi_trees_lib.pi_trees_lib import *

# other imports
import time


################################# SETUP VARIABLES ######################################


MIN_SIMULTANEOUS_ORDERS = 2   # Amigo won't fetch the drinks until he has this amount of requests
MAX_SIMULTANEOUS_ORDERS = 3   # Amigo will imediatly fetch the drinks when he has reached this amount of requests
MAX_DRINKS_SERVED       = 3   # The challenge will finish when Amigo has served this amount of requests

PENDING_DELIVERIES      = 0
PENDING_REQUEST         = 0
DELIVERED_DRINKS        = 0


#########################################################################################



class CocktailPartyBT():
    def __init__(self):

        pendingReq = 0;

        # The root node
        ROOT_NODE = Sequence("ROOT_NODE")
        

        ############## Get request from people ##############

        NAV_LIVING_ROOM_t = NavTo("NAV_LIVING_ROOM_t", "living room")
        GET_PERSON_REQ_t = GetPersonRequest("GET_PERSON_REQ_t")
        CHECK_REQ_1_t = CheckPendingRequests("CHECK_REQ_1_t", MIN_SIMULTANEOUS_ORDERS)
        # TAKE_REQUEST_sq = Sequence("TAKE_REQUEST_sq")
        GET_REQUESTS_sq = Sequence("GET_REQUESTS_sq")


        # TAKE_REQUEST_sq.add_child(CHECK_REQ_1_t)
        # TAKE_REQUEST_sq.add_child(GET_PERSON_REQ_t)
        
        # GET_REQUESTS_sq.add_child(CHECK_REQ_1_t)
        GET_REQUESTS_sq.add_child(NAV_LIVING_ROOM_t)
        # GET_REQUESTS_sq.add_child(TAKE_REQUEST_sq)
        GET_REQUESTS_sq.add_child(GET_PERSON_REQ_t)


        ############## Get drinks requested ##############
        
        GET_DRINKS_sq = Sequence("GET_DRINKS_sq")
        NAV_KITCHEN_t = NavTo("NAV_KITCHEN_t", "kitchen")
        FETCH_DRINK_t = FetchDrinks("FETCH_DRINK_t")
        CHECK_REQ_2_t = CheckPendingRequests("CHECK_REQ_2_t", MIN_SIMULTANEOUS_ORDERS)

        GET_DRINKS_sq.add_child(CHECK_REQ_2_t)
        GET_DRINKS_sq.add_child(NAV_KITCHEN_t)
        GET_DRINKS_sq.add_child(FETCH_DRINK_t)


        ############## Serve drinks to people ##############

        SERVE_DRINKS_sq = Sequence("SERVE_DRINKS_sq")
        DELIVER_DRINK_t = HandoverDrink("DELIVER_DRINK_t")
        CHECK_PENDING_DELIV_t = CheckPendingDeliveries("CHECK_PENDING_DELIV_t")

        SERVE_DRINKS_sq.add_child(CHECK_PENDING_DELIV_t)
        SERVE_DRINKS_sq.add_child(NAV_LIVING_ROOM_t)
        SERVE_DRINKS_sq.add_child(DELIVER_DRINK_t)



        ############## Main Tree ##############
        CHECK_DELIVERED_DRINKS_t = CheckDeliveredrinks("CHECK_DELIVERED_DRINKS_t", MAX_DRINKS_SERVED)
        COCKTAIL_PARTY = Selector("COCKTAIL_PARTY")

        COCKTAIL_PARTY.add_child(CHECK_DELIVERED_DRINKS_t)
        COCKTAIL_PARTY.add_child(SERVE_DRINKS_sq)
        COCKTAIL_PARTY.add_child(GET_DRINKS_sq)
        COCKTAIL_PARTY.add_child(GET_REQUESTS_sq)
        

        ROOT_NODE.add_child(COCKTAIL_PARTY)

        
        # Print a simple representation of the tree
        print "\nBehavior Tree Structure"
        print_tree(ROOT_NODE)

        print "\n\n Starting ROOT_NODE"
        
        it = 0
        # Run the tree
        while True:
            status = ROOT_NODE.run()
            print "\nIteration ", it
            it+=1
            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break


#########################################################################
#########################################################################

class HandoverDrink(Task):
    def __init__(self, name, *args, **kwargs):
        super(HandoverDrink, self).__init__(name, *args, **kwargs)
        
        self.name = name
        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global PENDING_DELIVERIES
        global DELIVERED_DRINKS

        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        if PENDING_DELIVERIES > 0:
            PENDING_DELIVERIES -= 1
            DELIVERED_DRINKS += 1
            print "[", self.name, "]\t", "Handing over a drink"

            print "[", self.name, "]\t", "Return SUCCESS"
            return TaskStatus.SUCCESS
        else:
            print "[", self.name, "]\t", "No drinks to deliver"
            print "[", self.name, "]\t", "Return FAILURE"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"


#########################################################################

class FetchDrinks(Task):
    def __init__(self, name, *args, **kwargs):
        super(FetchDrinks, self).__init__(name, *args, **kwargs)

        self.name = name
        self.count = 0
        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global PENDING_DELIVERIES
        global PENDING_REQUEST

        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        PENDING_DELIVERIES += 1
        PENDING_REQUEST -= 1
        print "[", self.name, "]\t", "Fetching a drink"
        
        print "[", self.name, "]\t", "Return SUCCESS"
        return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"

#########################################################################

class CheckDeliveredrinks(Task):
    def __init__(self, name, maxDeliveredN, *args, **kwargs):
        super(CheckDeliveredrinks, self).__init__(name, *args, **kwargs)     

        self.name = name
        self.maxDeliveredN = maxDeliveredN

        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global DELIVERED_DRINKS
        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        if DELIVERED_DRINKS >= self.maxDeliveredN:
            print "[", self.name, "]\t", "Delivered all the drinks needed"

            print "[", self.name, "]\t", "Return SUCCESS"
            return TaskStatus.SUCCESS
        else:
            print "[", self.name, "]\t", "Still need to deliver more drinks", DELIVERED_DRINKS, self.maxDeliveredN
            print "[", self.name, "]\t", "Return FAILURE"
            return TaskStatus.FAILURE
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"

#########################################################################

class CheckPendingRequests(Task):
    def __init__(self, name, minReqN, *args, **kwargs):
        super(CheckPendingRequests, self).__init__(name, *args, **kwargs)     

        self.name = name
        self.minReqN = minReqN

        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global PENDING_REQUEST

        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        if PENDING_REQUEST < self.minReqN:
            print "[", self.name, "]\t", "Number of requests is not enough:", PENDING_REQUEST, self.minReqN

            print "[", self.name, "]\t", "Return FAILURE"
            return TaskStatus.FAILURE
        else:
            self.count = 0
            print "[", self.name, "]\t", "Return SUCCESS"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"

#########################################################################

class CheckPendingDeliveries(Task):
    def __init__(self, name, *args, **kwargs):
        super(CheckPendingDeliveries, self).__init__(name, *args, **kwargs)
        
        self.name = name

        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global PENDING_DELIVERIES

        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        if PENDING_DELIVERIES > 0:
            print "[", self.name, "]\t", "Still have drinks to be delivered!", PENDING_DELIVERIES
            print "[", self.name, "]\t", "Return SUCCESS"
            return TaskStatus.SUCCESS
        else:
            print "[", self.name, "]\t", "No pending deliveries!", PENDING_REQUEST
            print "[", self.name, "]\t", "Return FAILURE"
            return TaskStatus.FAILURE
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"

#########################################################################

class NavTo(Task):
    def __init__(self, name, destination, *args, **kwargs):
        super(NavTo, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.destination = destination
        self.count = 0
        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        self.count = self.count + 1
        print "[", self.name, "]\t", "Navigating to the", self.destination

        # has the robot arrived at the living room?
        if self.count < 2:
            print "[", self.name, "]\t", "Return RUNNING"
            return TaskStatus.RUNNING
        else:
            self.count = 0
            print "[", self.name, "]\t", "Return SUCCESS"
            return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"

#########################################################################

class GetPersonRequest(Task):
    def __init__(self, name, *args, **kwargs):
        super(GetPersonRequest, self).__init__(name, *args, **kwargs)

        self.name = name

        print "[", self.name, "]\t", "Creating task"
 
    def run(self):
        global PENDING_REQUEST

        print "[", self.name, "]\t", "Run"
        #time.sleep(0.1)

        PENDING_REQUEST += 1

        print "[", self.name, "]\t", "John Doe asked for a coke"

        print "[", self.name, "]\t", "Return SUCCESS"
        return TaskStatus.SUCCESS
    
    def reset(self):
        print "[", self.name, "]\t", "Reset"


#########################################################################


if __name__ == '__main__':
    tree = CocktailPartyBT()