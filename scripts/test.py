# -*- coding: utf-8 -*-
import random
import time
import rospy
from cluedo.srv import Hints, Discard, Hypothesis, Compare
from std_msgs.msg import Bool


people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]
all_hints = people_list + weapons_list + places_list
# Murder
who = ""
what = ""
where = ""


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
    global who, what, where
    
    if len(people_list):
      who = random.choice(people_list)
      
    if len(weapons_list):
      what = random.choice(weapons_list)
      
    if len(places_list):
      where = random.choice(places_list)
      
    print("A murder is announced!")
    print(who,what,where)
   
    
    return who, what, where


def __consistent_hypothesis():
    hint = Hint()
    if len(people_list):
        hint.hint0 = random.choice(people_list)
    if len(weapons_list):
        hint.hint1 = random.choice(weapons_list)
    if len(people_list):
        hint.hint2 = random.choice(places_list)

    return hint
    

def __inconsistent_hypothesis():
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
    rand = random.randint(0,1)
    if rand == 0:
        hint = __consistent_hypothesis()
        print('CONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    else:
        hint = __inconsistent_hypothesis()
        print('INCONSISTENT:', hint.hint0, hint.hint1, hint.hint2, hint.hint3)
    
    return hint.hint0, hint.hint1, hint.hint2, hint.hint3


def remove_discarded(item):
    global people_list, weapons_list, places_list, hints
    
    if item in people_list:
        people_list = [item]
        print(people_list)
    elif item in weapons_list:
        weapons_list = [item]
        print(weapons_list)
    elif item in places_list:
        places_list = [item]
        print(places_list)
    hints = people_list + weapons_list + places_list
    print(hints)
    
    return[]
    
            

def verify_solution(req):
    global who, what, where
    print('------------verify solution --------------')
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
    rospy.Service('remove_hypothesis', Discard, remove_discarded)
    rospy.Service('generate_murder',  Hypothesis, gen_murder)
    rospy.Service('verify_solution', Compare, verify_solution)

    print('ready')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
    
if __name__ == '__main__':
    main()
