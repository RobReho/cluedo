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
from cluedo.srv import Hypothesis, Discard, Hints
from os.path import dirname, realpath
import random

srv_client_get_hint_ = rospy.ServiceProxy('/get_hint', Hints)
srv_client_remove_discarded_ = rospy.ServiceProxy('/remove_discarded', Discard)

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
    
            
            
    def make_hypothesis(self,hyp,hypID):
        ##
        #\brief Insert an hypothesis in the system
        #@param hypothesis the hypothesis that will be inserted in the system, which is of type hypothesis
        objProp = ['who','what','here']
        class_ = ['PERSON','WEAPON','PLACE']
        
        for i in range(len(hyp)-1):
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'ontology'
                req.command= 'ADD'
                req.primary_command_spec= 'OBJECTPROP'
                req.secondary_command_spec= 'IND'
                req.args= [objProp[i], hypID, hyp[i]]
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
                req.args= [hyp[i],class_[i]]
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
 
    
    def retrieve_dataPropriety(self,dataProp,ind):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'QUERY'
            req.primary_command_spec= 'DATAPROP'
            req.secondary_command_spec= 'IND'
            req.args= [dataProp,ind]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
    
    
    def add_dataPropriety(self,dataProp,ind,_type,value):
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'ADD'
            req.primary_command_spec= 'DATAPROP'
            req.secondary_command_spec= 'IND'
            req.args= [dataProp,ind,_type,value]
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
            
            
  
def main():
    rospy.init_node('ontology_client')
    armor = Armor_communication()
    armor.__init__()
    armor.log_to_file()
    armor.load_file()
#    ind = 0
#    hypID = 'HP'+str(ind)
    hint_list = []
    print('ready')
    hp = srv_client_get_hint_()
    print(hp)
    hint_list.append(hp.hint0)
    hint_list.append(hp.hint1)
    hint_list.append(hp.hint2)
    hint_list.append(hp.hint3)

    armor.make_hypothesis(hint_list,'HP_0')
    armor.reason()
    print(armor.retrieve_class('HYPOTHESIS'))
    
    
    
    
    #srv_client_remove_discarded_()
#    armor.add_dataPropriety('isSuspect','Scarlet','boolean','true')
#    rospy.sleep(2)
#    print(armor.retrieve_dataPropriety('isSuspect','Green'))
#    print(armor.retrieve_dataPropriety('isSuspect','Scarlet'))
    
#    while len(armor.retrieve_class('PLACE')) != 0:
#        place_list = armor.retrieve_class('PLACE')
#        print(place_list)
#        place = random.choice(place_list)
#        print(place)
#        armor.remove_instance(place,'PLACE')
#        armor.reason()
#        rospy.sleep(1)
        
    
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
