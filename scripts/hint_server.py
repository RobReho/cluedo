# -*- coding: utf-8 -*-
"""@package hint_server
This node stores the solution of the game,
generates hints to give to the player
and handles the incoming hypothesis by comparing them
withe the stored solution.

"""
import random
import rospy
from cluedo.srv import Hints, Hypothesis, Compare
from std_msgs.msg import String


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


def ui_message(message):
    """
    \brief Handles debugging msgs and publishes UI output
    @param String message
    """
    rospy.loginfo(message)
    ui.publish(rospy.get_caller_id() + ':                 ' + message)
    

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
      
    ui_message("A murder is announced!")
    print(who,what,where)
    
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
    ui_message("Here's a hint...")
    rand = random.randint(0,1)
    if rand == 0:
        hint = __consistent_hypothesis()
        print('CONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    else:
        hint = __inconsistent_hypothesis()
        print('INCONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    
    return hint.hint0, hint.hint1, hint.hint2, hint.hint3

            

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
    global who, what, where, people_list, weapons_list, places_list
    ui_message("Let's verify this hypothesis")
    print(req.person, req.weapon, req.place)
    same_person = False
    same_weapon = False
    same_place = False
    if req.person == who:
        same_person = True
    else:
        people_list.remove(req.person)
        
    if req.weapon == what:
        same_weapon = True
    else:
        weapons_list.remove(req.weapon)
        
    if req.place == where:
        same_place = True
    else:
        places_list.remove(req.place)
        
    print(same_person, same_weapon, same_place)
    
    return same_person,same_weapon,same_place
          
  
def main():
    rospy.init_node('oracle')

    # service server to give a hint
    rospy.Service('get_hint', Hints, give_hint)
    # service server to generate the right hypothesis
    rospy.Service('generate_murder',  Hypothesis, gen_murder)
    # service server to compare an hypothesis with the right one
    rospy.Service('verify_solution', Compare, verify_solution)

    print('ready')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
    
if __name__ == '__main__':
    main()
