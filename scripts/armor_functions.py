#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 09:11:17 2022

@author: root
"""
import rospy
from std_srvs.srv import *
from armor_msgs.msg import * 
from armor_msgs.srv import * 
from cluedo.srv import Hypothesis, Discard
from os.path import dirname, realpath
import random

class Hypothesis_():
    ##
    #\class hypothesis
    #\brief struct to handle hypothesis made of person,place,weapon and hypothesis_code
    def __init__(self):
        ##
        #\brief init function to initialize the class
        super(Hypothesis, self).__init__()
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


    def log_to_file(self):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'LOG'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= 'ON'
            req.args= ['/root/ros_ws/src/cluedo/armor.log']
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
            
            
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
            
          
    def data_property(self,item):
        ##
        #\brief load the owl file
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'DATAPROP'
            req.args= [item]
            
            msg = self.armor_service(req)

        except rospy.ServiceException as e:
            print(e)  
    
            
            
    def make_hypothesis(self,person,weapon,place):
        ##
        #\brief Insert an hypothesis in the system
        #@param hypothesis the hypothesis that will be inserted in the system, which is of type hypothesis
        

        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'OBJECTPROP'
            req.secondary_command_spec= 'IND'
            req.args= ['who','HP',person]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)

        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [person,'PERSON']
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
            req.args= ['what','HP',weapon]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)

        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [weapon,'WEAPON']
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
            req.args= ['where','HP',place]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)

        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [place,'PLACE']
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
        
        
        
    def remove_instance(self,item,_class):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'REMOVE'
            req.primary_command_spec= 'IND'
            req.secondary_command_spec= 'CLASS'
            req.args= [item,_class]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
 
  
def main():
    rospy.init_node('ontology_client')
    armor = Armor_communication()
    armor.__init__()
    armor.log_to_file()
    armor.load_file()
    
    place_list = armor.retrieve_class('PLACE')
    print(place_list)
    place = random.choice(place_list)
    print(place)
    armor.remove_instance(place,'PLACE')
    armor.reason()
    rospy.sleep(1)
    remaning = armor.retrieve_class('PLACE')
    print(remaning)
    
#    person = random.choice(pe)
#    
#    we = armor.retrieve_class('WEAPON')
#    weapon = random.choice(we)
#    
#    hp = [person,weapon,place]
#    print(hp)
#    
#    armor.make_hypothesis(person,weapon,place)
#    
#    hypo = armor.retrieve_class('HYPOTHESIS')
#    print("retrieved: ",hypo)
    
    
    
    rospy.sleep(1)
    



if __name__ == "__main__":
    main()
