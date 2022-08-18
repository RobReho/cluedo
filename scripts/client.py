import rospy
import random
from std_srvs.srv import *
from cluedo.srv import Hypothesis, Discard

def main():
  srv_client_random_hypothesis_ = rospy.ServiceProxy('/random_hypothesis', Hypothesis)  
  srv_client_remove_hypothesis_ = rospy.ServiceProxy('/remove_hypothesis', Discard)  

  rospy.init_node('client')
  rospy.wait_for_service("random_hypothesis") 

  #while(1):
  hyp = srv_client_random_hypothesis_()
  print("My hypothesis is ", hyp.who, hyp.what, hyp.where)
    
    
    #srv_client_remove_hypothesis_(hyp.who)
    #print(hyp.who)
    #srv_client_remove_hypothesis_(hyp.what)
    #print(hyp.what)
    #srv_client_remove_hypothesis_(hyp.where)
    #print(hyp.where)
    
  rospy.sleep(1)
    



if __name__ == "__main__":
    main()
