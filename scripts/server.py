import rospy
import random
from std_srvs.srv import *
from cluedo.srv import Hypothesis, Discard


people_list = ["001","002","003","004","005","006"]
weapons_list = ["007","008","009","010","011","012"]
places_list = ["013","014","015","016","017","018","019","020","021"]
# Murder
who = ""
what = ""
where = ""
# Hints possessed by each person
ID1 = []
ID2 = []
ID3 = []
ID4 = []
ID5 = []
ID6 = []
source_ID = [ID1,ID2,ID3,ID4,ID5,ID6]

def distr_hints():
    hints = people_list + weapons_list + places_list
    
    for i in range(len(source_ID)):
      for j in range (0, 3):
        source_ID[i].append(random.choice(hints))
      print(source_ID[i])
    
    
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



def remove_discarded(item):
    if item in people_list:
      print("hvi")
      people_list.remove(item)
      print("item ",item,"has been removed")
    elif item in weapons_list:
      weapons_list.remove(item)
      print("item ",item,"has been removed")
    elif item in places_list:
      places_list.remove(item)
      print("item ",item,"has been removed")
      
    return[]
      
      
def main():
    rospy.init_node('generate_hypothesis')
    
    rospy.Service('random_hypothesis', Hypothesis, gen_murder)
    rospy.Service('remove_hypothesis', Discard, remove_discarded)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
