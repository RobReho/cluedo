# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import *
from armor_msgs.msg import * 
from armor_msgs.srv import * 
from cluedo.srv import Hypothesis, Discard, Hints
from os.path import dirname, realpath
import random

people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard room","kitchen","dining room","hall","study"]

class hypothesis():
    ##
    #\class hypothesis
    #\brief struct to handle hypothesis made of person,place,weapon and hypothesis_code
    def __init__(self):
        ##
        #\brief init function to initialize the class
        super(hypothesis, self).__init__()
        self.person=""
        self.place=""
        self.weapon=""
        self.hypothesis_code="HP-1"
        
        
class Armor_communication():
    ##
    #\class Armor_Communication
    def __init__(self):
        ##
        #\brief Initilize the class by declaring the client to "armor_interface_srv" service
        super(Armor_communication, self).__init__()
        rospy.wait_for_service('armor_interface_srv')
        self.armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)            
            
    def reason(self):
        ##
        #\brief Make the armor system reason
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'REASON'
            req.primary_command_spec= ''
            req.secondary_command_spec= ''
            req.args= []
            msg = self.armor_service(req)
        except rospy.ServiceException as e:
            print(e)
            
            
    def load_file(self):
        ##
        #\brief load the owl file
        try:
            path = dirname(realpath(__file__))
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'LOAD'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= ''
            req.args= [path + '/../cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
            
            msg = self.armor_service(req)

        except rospy.ServiceException as e:
            print(e)
            
    def retrieve_class(self,cls):
        ##
        #\brief Obtain the list of all the places inside the system
        #@return The array requested
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [cls]
            msg = self.armor_service(req)
            queries=msg.armor_response.queried_objects
            cont=0
            A=[0]*len(queries)
            for query in queries:
                results=query[40:]
                results=results[:len(results)-1]
                #print(results)
                A[cont]=results
                cont=cont+1
            return A
        except rospy.ServiceException as e:
            print(e)
            
            
    def make_hypothesis(self,person,weapon,place,hyp_code):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['who',hyp_code,person]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
            
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['what',hyp_code,weapon]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)


        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['where',hyp_code,place]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)

        self.reason()
    
    
        
    def details_of_an_hold_hypothesis(self,hypothesis_code):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['who',hypothesis_code]
            msg = self.armor_service(req)
            print(msg.armor_response.queried_objects)
            
        except rospy.ServiceException as e:
            print(e)
            
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['what',hypothesis_code]
            msg = self.armor_service(req)
            print(msg.armor_response.queried_objects)

        except rospy.ServiceException as e:
            print(e)
            
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['where',hypothesis_code]
            msg = self.armor_service(req)
            print(msg.armor_response.queried_objects)

        except rospy.ServiceException as e:
            print(e)

    
    

def main():
    rospy.init_node('armor_client')
    armor = Armor_communication()
    armor.__init__()
    armor.load_file()

    n_hyp = 0
    hypothesis_code = 'HP'+str(n_hyp)

    for i in range(0,4):
        person = random.choice(people_list)
        weapon = random.choice(weapons_list)
        place = random.choice(places_list)
        print(person,weapon,place)
        armor.make_hypothesis(person, weapon, place, hypothesis_code)
        print(armor.retrieve_class('HYPOTHESIS'))
        print(armor.retrieve_class('COMPLETED'))
        print(armor.retrieve_class('INCONSISTENT'))
        print(armor.details_of_an_hold_hypothesis(hypothesis_code))
        n_hyp = n_hyp +1
        hypothesis_code = 'HP'+str(n_hyp)
        rospy.sleep(2)
        
    rospy.sleep(1)
    



if __name__ == "__main__":
    main()
