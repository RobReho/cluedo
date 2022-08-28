# -*- coding: utf-8 -*-
import random
import time
import rospy
from cluedo.srv import Hints, Discard

people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]
hints = people_list + weapons_list + places_list

class Hint:
    def __init__(self):
        ##
        #\brief init function to initialize the class
        super(Hint, self).__init__()
        self.hint0 = ""
        self.hint1 = ""
        self.hint2 = ""
        self.hint3 = ""
        
        
def consistent_hypothesis():
    hint = Hint()
    if len(people_list):
        hint.hint0 = random.choice(people_list)
    if len(weapons_list):
        hint.hint1 = random.choice(weapons_list)
    if len(people_list):
        hint.hint2 = random.choice(places_list)
#    hyp.append(person)
#    hyp.append(weapon)
#    hyp.append(place)
    return hint
    

def inconsistent_hypothesis():
    hint = Hint()
    range_ = random.randint(1,4)
    print(range_)
    for i in range(range_):
        if i == 0:
            hint.hint0 = random.choice(hints)
        elif i == 1:
            hint.hint1 = random.choice(hints)
        elif i == 2:
            hint.hint2 = random.choice(hints)
        elif i == 3:
            hint.hint3 = random.choice(hints)
    return hint
        

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
    
    
def give_hint(self):
    rand = random.randint(0,1)
    hint = []
    if rand == 0:
        hint = consistent_hypothesis()
    else:
        hint = inconsistent_hypothesis()
    
    return hint.hint0, hint.hint1, hint.hint2, hint.hint3
            
            
def main():
    rospy.init_node('oracle')

    rospy.Service('get_hint', Hints, give_hint)
    rospy.Service('remove_hypothesis', Discard, remove_discarded)

    print('ready')

#    while 1:
#        hint = consistent_hypothesis()
#        print(hint.hint0, hint.hint1, hint.hint2, hint.hint3)
#        
#        inc_hint = inconsistent_hypothesis()
#        print(inc_hint.hint0, inc_hint.hint1, inc_hint.hint2, inc_hint.hint3)
#    print(people_list)
#    print(weapons_list)
#    print(places_list)
#    print(hints)
#    time.sleep(2)
#    remove_discarded('Scarlet')
#    time.sleep(2)
#    remove_discarded('hall')
#    time.sleep(2)
#    remove_discarded('dagger')
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
    
if __name__ == '__main__':
    main()
