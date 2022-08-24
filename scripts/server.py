import rospy
import random
from std_srvs.srv import *
from cluedo.srv import Hypothesis, Discard, Compare


people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard room","kitchen","dining room","hall","study"]
# Murder
who = ""
what = ""
where = ""
# Hints possessed by each person
ID1 = []
ID2 = []
ID3 = []
ID0 = []
source_ID = [ID0,ID1,ID2,ID3]
tourn_counter = 0

def distr_hints():
    hints = people_list + weapons_list + places_list
    random.shuffle(hints)
    
    while hints:
        for i in range (0, 4):
            if hints:
                source_ID[i].append( hints.pop() )  
        
    print(ID0)
    print(ID1)
    print(ID2)
    print(ID3)
    
    
def gen_murder(req):
    global who, what, where
    
    if len(people_list):
      who = random.choice(people_list)
      people_list.remove(who)
      
    if len(weapons_list):
      what = random.choice(weapons_list)
      weapons_list.remove(what)
      
    if len(places_list):
      where = random.choice(places_list)
      places_list.remove(where)
      
    print("A murder is announced!")
    print(who,what,where)
    distr_hints()
    
    return who, what, where


def deliver_hint(req):
    global tourn_counter
    req.person
    req.weapon
    req.place
    same_person = False
    same_weapon = False
    same_place = False
    print('your hyp is:',req.person,req.weapon,req.place)
    print('source:',tourn_counter)

    if tourn_counter == 0:
        same_person = req.person in ID0
        same_weapon = req.weapon in ID0
        same_place = req.place in ID0
        print(same_person,same_weapon,same_place)
        
    elif tourn_counter == 2:
        same_person = req.person in ID1
        same_weapon = req.weapon in ID1
        same_place = req.place in ID1
        print(same_person,same_weapon,same_place)

    elif tourn_counter == 3:
        same_person = req.person in ID2
        same_weapon = req.weapon in ID2
        same_place = req.place in ID2
        print(same_person,same_weapon,same_place)
        
    elif tourn_counter == 4:
        same_person = req.person in ID3
        same_weapon = req.weapon in ID3
        same_place = req.place in ID3
        print(same_person,same_weapon,same_place)
        
    tourn_counter = (tourn_counter + 1)%4
    return same_person,same_weapon,same_place


def verify_solution(req):
    global who, what, where
    same_person = False
    same_weapon = False
    same_place = False
    if req.person == who:
        same_person = True
    if req.weapon == what:
        same_weapon = True
    if req.place == where:
        same_place = True
        
    return same_person,same_weapon,same_place
        

    
#def remove_discarded(item):
#    if item in people_list:
#      print("hvi")
#      people_list.remove(item)
#      print("item ",item,"has been removed")
#    elif item in weapons_list:
#      weapons_list.remove(item)
#      print("item ",item,"has been removed")
#    elif item in places_list:
#      places_list.remove(item)
#      print("item ",item,"has been removed")
#      
#    return[]
      
      
def main():
    rospy.init_node('oracle')
    
    rospy.Service('generate_murder', Hypothesis, gen_murder)
#    rospy.Service('remove_hypothesis', Discard, remove_discarded)
    rospy.Service('query_source', Compare, deliver_hint)
    rospy.Service('verify_solution', Compare, verify_solution)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
