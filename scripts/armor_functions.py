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
from cluedo.srv import Hypothesis, Discard, Hints, Compare
from os.path import dirname, realpath
import random

people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]

srv_client_get_hint_ = rospy.ServiceProxy('/get_hint', Hints)
srv_client_remove_discarded_ = rospy.ServiceProxy('/remove_discarded', Discard)
srv_client_query_source_ = rospy.ServiceProxy('/query_source', Compare)
srv_client_final_hypothesis_ = rospy.ServiceProxy('/verify_solution', Compare)
srv_client_generate_murder_ = rospy.ServiceProxy('/generate_murder', Hypothesis)  

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
        
        for i in range(len(hyp)):
            try:
                req=ArmorDirectiveReq()
                req.client_name= 'cluedo'
                req.reference_name= 'ontology'
                req.command= 'ADD'
                req.primary_command_spec= 'OBJECTPROP'
                req.secondary_command_spec= 'IND'  
                if hyp[i] in people_list:
                    
                    req.args= ['who', hypID, hyp[i]]
                elif hyp[i] in weapons_list:
                   
                    req.args= ['what', hypID, hyp[i]]
                elif hyp[i] in places_list:
                   
                    req.args= ['where', hypID, hyp[i]]
                print(hyp[i])
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
                if hyp[i] in people_list:
                    req.args= [hyp[i],'PERSON']
                elif hyp[i] in weapons_list:
                    req.args= [hyp[i],'WEAPON']
                elif hyp[i] in places_list:
                    req.args= [hyp[i],'PLACE']
                
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
#    hint_list = []
#    print('ready')
#    hp = srv_client_get_hint_()
#    print(hp)
#    hint_list.append(hp.hint0)
#    hint_list.append(hp.hint1)
#    hint_list.append(hp.hint2)
#
#    armor.make_hypothesis(hint_list,'HP_0')
#    armor.reason()
#    print(armor.retrieve_class('HYPOTHESIS'))
    hyp = []
    n_hyp = 0
    hypothesis_code = 'HP'+str(n_hyp)
    
    for i in range(0,4):
        person = random.choice(people_list)
        weapon = random.choice(weapons_list)
        place = random.choice(places_list)
        plac = random.choice(places_list)
        hyp = [person,weapon, place,plac]
        print(hypothesis_code)
        print(hyp)
        armor.make_hypothesis(hyp, hypothesis_code)
        armor.reason()
#        print(armor.retrieve_class('HYPOTHESIS'))
        print(armor.retrieve_class('COMPLETED'))
        print(armor.retrieve_class('INCONSISTENT'))
        #print(armor.details_of_an_hold_hypothesis(hypothesis_code))
        n_hyp = n_hyp +1
        hypothesis_code = 'HP'+str(n_hyp)
        rospy.sleep(2)
    
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
