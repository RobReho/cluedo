# -*- coding: utf-8 -*-
"""@package hint_server
This node stores the solution of the game,
generates hints to give to the player
and handles the incoming hypothesis

"""
import random
import time
import rospy
from cluedo.srv import Hints, Discard, Hypothesis, Compare
from std_msgs.msg import Bool, String


people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestick","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]
all_hints = people_list + weapons_list + places_list
# Murder
who = ""
what = ""
where = ""
ui = rospy.Publisher('ui_output', String, queue_size=30)


class Hint:
    def __init__(self):
        ##
        #\brief init function to initialize the class
        super(Hint, self).__init__()
        self.hint0 = ""
        self.hint1 = ""
        self.hint2 = ""
        self.hint3 = ""



def gen_murder(req):
    ''' \brief Initialize the game by randomly generating a murderer, weapon and place
    Callback of the 'generate_murder' service
    Custom service: Murder.srv
        ---
        string who
        string what
        string where
    @param[in] req (empty)
    @return who, what, where - saved globally 
'''
    global who, what, where
    
    if len(people_list):
      who = random.choice(people_list)
      
    if len(weapons_list):
      what = random.choice(weapons_list)
      
    if len(places_list):
      where = random.choice(places_list)
      
    print("A murder is announced!")
    print(who,what,where)
    ui.publish(rospy.get_caller_id() +"A murder is announced!")
    
    return who, what, where



def __consistent_hypothesis():
    ''' \brief Genetares a consisten hypothesys from the lists of people, weapons and places available
    Function called in give_hint()
    
    @return hint instance
'''
    hint = Hint()
    if len(people_list):
        hint.hint0 = random.choice(people_list)
    if len(weapons_list):
        hint.hint1 = random.choice(weapons_list)
    if len(people_list):
        hint.hint2 = random.choice(places_list)

    return hint

    

def __inconsistent_hypothesis():
    ''' \brief Genetares an inconsisten hypothesys from the lists of people, weapons and places available
    Function called in give_hint()
    
    @return hint instance
'''
    hint = Hint()
    range_ = random.randint(2,4)
    for i in range(range_):
        if i == 0:
            hint.hint0 = random.choice(people_list)
        elif i == 1:
            hint.hint1 = random.choice(weapons_list)
        elif i == 2:
            hint.hint2 = random.choice(places_list)
        elif i == 3:
            hint.hint3 = random.choice(all_hints)
            
    return hint
        


def give_hint(self):
    ''' \brief Randomly chooses a consostent or inconsistent hint to give to the player
    Callback of the 'get_hint' service
    Custom service: Hints.srv
        ---
        string hint0
        string hint1
        string hint2
        string hint3
    
    @return array of hints
'''
    ui.publish("Here's a hint...")
    rand = random.randint(0,1)
    if rand == 0:
        hint = __consistent_hypothesis()
        print('CONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    else:
        hint = __inconsistent_hypothesis()
        print('INCONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    
    return hint.hint0, hint.hint1, hint.hint2, hint.hint3


#def remove_discarded(item):
#    global people_list, weapons_list, places_list, hints
#    
#    if item in people_list:
#        people_list = [item]
#        print(people_list)
#    elif item in weapons_list:
#        weapons_list = [item]
#        print(weapons_list)
#    elif item in places_list:
#        places_list = [item]
#        print(places_list)
#    hints = people_list + weapons_list + places_list
#    print(hints)
#    
#    return[]
    
            

def verify_solution(req):
    ''' \brief Compare the hypothesis of the player to the solution stored globally
    Callback of the 'verify_solution' service
    Custom service: Compare.srv
        string person
        string weapon
        string place
        ---
        bool person
        bool weapon
        bool place
    @param[in] req: string person, string weapon, string place
    @return bool: true if the the elements are the same
'''
    global who, what, where
    ui.publish(rospy.get_caller_id() +"Let's verify this hypothesis")
    print(req.person, req.weapon, req.place)
    same_person = False
    same_weapon = False
    same_place = False
    if req.person == who:
        same_person = True
    if req.weapon == what:
        same_weapon = True
    if req.place == where:
        same_place = True
    print(same_person, same_weapon, same_place)
    
    return same_person,same_weapon,same_place
          
  
def main():
    rospy.init_node('oracle')

    rospy.Service('get_hint', Hints, give_hint)
    #rospy.Service('remove_hypothesis', Discard, remove_discarded)
    rospy.Service('generate_murder',  Hypothesis, gen_murder)
    rospy.Service('verify_solution', Compare, verify_solution)

    print('ready')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
    
if __name__ == '__main__':
    main()
