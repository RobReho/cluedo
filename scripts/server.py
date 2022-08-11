import rospy
import random
from std_srvs.srv import *
from cluedo.srv import Hypothesis, Discard


people_list = ["001","002","003","004","005","006"]
weapons_list = ["007","008","009","010","011","012"]
places_list = ["013","014","015","016","017","018","019","020","021"]
who = ""
what = ""
where = ""

def gen_hypothesis(req):
    global who, what
    print("Generating hypothesis...")
    if len(people_list):
      who = random.choice(people_list)
      print("who:",who)
    else:
      print("no more suspects in the list")
      
    if len(weapons_list):
      what = random.choice(weapons_list)
      print("what:",what)
    else:
      print("no more weapons in the list")
      
    if len(places_list):
      where = random.choice(places_list)
      print("where:",where)
    else:
      print("no more places in the list")
      
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
    
    srv1 = rospy.Service('random_hypothesis', Hypothesis, gen_hypothesis)
    srv2 = rospy.Service('remove_hypothesis', Discard, remove_discarded)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
