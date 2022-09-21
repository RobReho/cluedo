#!/usr/bin/env python3
"""@package cluedo state machine
This node handles the states of the FSM

smach viewer:
    rosrun smach_viewer smach_viewer.py
    
armore server:
    rosrun armor execute it.emarolab.armor.ARMORMainService
"""
import rospy
import smach
import smach_ros
import random
from std_msgs.msg import Int32, String

from armor_msgs.msg import * 
from armor_msgs.srv import * 
from cluedo.srv import Hypothesis, Discard, Hints, Compare
from os.path import dirname, realpath

# lists of peolpe, weapons and places to be retrieved from Armor server
people_list = []
weapons_list = []
places_list = []

srv_client_query_source_ = rospy.ServiceProxy('/query_source', Compare)
srv_client_final_hypothesis_ = rospy.ServiceProxy('/verify_solution', Compare)
srv_client_generate_murder_ = rospy.ServiceProxy('/generate_murder', Hypothesis)  
srv_client_get_hint_ = rospy.ServiceProxy('/get_hint', Hints)
ui = rospy.Publisher('ui_output', String, queue_size=30)    # user interface publisher
n_hyp = 0

#------------------------ARMOR COMMUNICATION----------------------------------
class Armor_communication():
    """Armor communication class
 
    the class defines all the functions that allow the communcation 
    with the Armor server
    """
    def __init__(self):
        """
        \brief Initilize the class by declaring the client to "armor_interface_srv" service
        """
        super(Armor_communication, self).__init__()
        rospy.wait_for_service('armor_interface_srv')
        self.armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)


    def load_file(self):
        """
        \brief loads the owl file
        """
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
            
    def log_to_file(self):
        """
        \brief logs to a file in the same folder
        """
        try:
            req=ArmorDirectiveReq()
            req.client_name= 'cluedo'
            req.reference_name= 'ontology'
            req.command= 'LOG'
            req.primary_command_spec= 'FILE'
            req.secondary_command_spec= 'ON'
            req.args= ['/root/ros_ws/src/cluedo/armor.txt']
            msg = self.armor_service(req)
            
        except rospy.ServiceException as e:
            print(e)
            
    def reason(self):
        """
        \brief Makes the armor system reason
        """
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
            
    def retrieve_class(self,cls):
        """
        \brief Obtain the list of all the places inside the system
        @param cls Class to be retrived
        @return The array requested
        """
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
            
        
        
    def make_hypothesis(self,hyp,hypID):
        """
        \brief Create an hypothesis in the system
        @param hyp the hypothesis that will be inserted in the system, which is an array
            of various lenght
        @param hypID ID of the hypothesis to be created
        """
        c = 0
        # counts the hints in the hypothesis list
        for i in range(len(hyp)):
            if hyp[i]:
                c = c+1
        
        for i in range(c):
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
                self.armor_service(req)
                
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
                self.armor_service(req)
                

            except rospy.ServiceException as e:
                print(e)
        print('done')
                
                
                

        
#    def details_of_an_hold_hypothesis(self,hypothesis_code):
#        
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'QUERY'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['who',hypothesis_code]
#            msg = self.armor_service(req)
#            print(msg.armor_response.queried_objects)
#            
#        except rospy.ServiceException as e:
#            print(e)
#            
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'QUERY'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['what',hypothesis_code]
#            msg = self.armor_service(req)
#            print(msg.armor_response.queried_objects)
#
#        except rospy.ServiceException as e:
#            print(e)
#            
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'QUERY'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['where',hypothesis_code]
#            msg = self.armor_service(req)
#            print(msg.armor_response.queried_objects)
#
#        except rospy.ServiceException as e:
#            print(e)
#            
#            
#            
#    def remove_instance(self,item,_class):
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'REMOVE'
#            req.primary_command_spec= 'IND'
#            req.secondary_command_spec= 'CLASS'
#            req.args= [item,_class]
#            msg = self.armor_service(req)
#            
#        except rospy.ServiceException as e:
#            print(e)
            

armor = Armor_communication()

def ui_message(message):
    rospy.loginfo(message)
    ui.publish(rospy.get_caller_id() + ':   ' + message)
    
#-------------------------------- State machine classes---------------------------------
'''
# define state INITIALIZATION
Establish the communication with Armor server
Loads OWL file
Calls "generate murder" service to start the game
Retrieves people, weapons and places list from the OWL
'''
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready','err'])

    def execute(self, userdata):
        global people_list, weapons_list, places_list
        
        rospy.wait_for_service("armor_interface_srv") 
        # Init OWL file from Armor server
        armor.__init__()
        armor.log_to_file()
        armor.load_file()
        # retrieve OLW lists
        people_list = armor.retrieve_class('PERSON')
        weapons_list = armor.retrieve_class('WEAPON')
        places_list = armor.retrieve_class('PLACE')
        # Generate murder
        rospy.wait_for_service("generate_murder") 
        srv_client_generate_murder_()
        ui_message('Initializing game:')
        if people_list and weapons_list and places_list:
            ui_message('Detective ready')
            return 'ready'
        else:
            ui_message('places list failed to load')
            return 'err'
    
    
'''
# define state EXPLORE
Retrieves the list of available places
Randomly choose one
Reaches place
'''
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['err','found_source'])

    def execute(self, userdata):
        global places_list
        ui_message('EXPLORING')
    
        if places_list:
            place = random.choice(places_list)
            ui_message('I am going to the' + place)
            rospy.sleep(3)  # Reach place
            return 'found_source'
        else:
            rospy.logerr('places list failed to load')
            return 'err'
            
 

'''
# define state CHECK CONSISTENCY
Retrieve people and weapons
Choose random person an weapon
Make a new hypothesis
Check its consistency
(if inconsistent, delete and make another)
'''
class Make_hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['consistent','inconsistent'],
                                    input_keys=['person','weapon','place'],
                                    output_keys=['person','weapon','place'])

    def execute(self, userdata):
        global n_hyp
        ui_message('MAKING AN HYPOTHESIS:')
        # debug
#        if armor.retrieve_class('COMPLETED'):
#            for i in range(len(armor.retrieve_class('COMPLETED'))):
#                print(armor.details_of_an_hold_hypothesis(armor.retrieve_class('COMPLETED')[i]))
                
        hyp = srv_client_get_hint_()    # receive hint from hints server    
        hyp_list = [hyp.hint0,hyp.hint1,hyp.hint2,hyp.hint3]
            
        hypothesis_code = 'HP'+str(n_hyp)   # progressive number of hypothesis
#        print(hypothesis_code)
        armor.make_hypothesis(hyp_list, hypothesis_code)
        armor.reason()
        compl = armor.retrieve_class('COMPLETED')
        incons = armor.retrieve_class('INCONSISTENT')        

        n_hyp = n_hyp +1
        rospy.sleep(3)

        if hypothesis_code in compl and hypothesis_code not in incons: 
            userdata.person = hyp.hint0
            userdata.weapon = hyp.hint1
            userdata.place = hyp.hint2
            ui_message('The hypothesis is consistent')
            return 'consistent' 
        else:
            ui_message('The hypothesis is inconsistent')
            return 'inconsistent'
        
        
        
    
'''
# define state REACH ORACLE
Reach for the oracle
'''
class Reach_oracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','reached'],
                             input_keys=['person','weapon','place'],
                             output_keys=['person','weapon','place'])

    def execute(self, userdata):
        ui_message('REACHING ORACLE')
        rospy.sleep(3)  # reaching the oracle
        
        return 'reached'
    # return 'failed' # to be implemented in further iteration
        
    
        
# define state DELIVER HYPOTHESIS
class Deliver_hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','succeded'],
                             input_keys=['person','weapon','place'],
                             output_keys=['person','weapon','place'])

    def execute(self, userdata):
        ui_message('DELIVERING HYPOTHESIS')
        ui.publish(rospy.get_caller_id() +'Executing state DELIVER HYPOTHESIS')
        if userdata.person and userdata.weapon and userdata.place:
            ui_message('Hypothesis: It was ' + userdata.person + ' with the ' + userdata.weapon +' in the ' + userdata.place)
            return 'succeded'
        else:
            ui_message('The hypothesis has failed to be delivered')
            return 'failed'
        

# define state HYPOTHESIS CHECK 
class Hypothesis_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right','wrong'],
                             input_keys=['person','weapon','place'])

    def execute(self, userdata):
        ui_message('CHECKING HYPOTHESIS')
        rospy.sleep(3)
        resp = srv_client_final_hypothesis_(userdata.person, userdata.weapon, userdata.place)
        
        if resp.person and resp.weapon and resp.place is True:
            ui_message("The hypothesis is right!")
            return 'right'
        else:
            ui_message("The hypothesis is wrong")
            return 'wrong'


def main():
    rospy.init_node('cluedo_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['end'])
    sm_top.userdata.place = ''
    sm_top.userdata.person = ''
    sm_top.userdata.weapon = ''

    # Open the container
    with sm_top:

        smach.StateMachine.add('INIT', Initialization(),
                               transitions={'ready':'EXPLORE',
                                            'err':'INIT'})
    
        smach.StateMachine.add('EXPLORE', Explore(),
                               transitions={'found_source':'MAKE HYPOTHESIS',
                                            'err':'INIT'})
        
        smach.StateMachine.add('MAKE HYPOTHESIS', Make_hypothesis(),
                               transitions={'consistent':'REACH ORACLE',
                                            'inconsistent':'EXPLORE'})

       

            # Add states to the container
        smach.StateMachine.add('REACH ORACLE', Reach_oracle(), 
                                   transitions={'failed':'REACH ORACLE', 
                                                'reached':'DELIVER HYPOTHESIS'})
        smach.StateMachine.add('DELIVER HYPOTHESIS', Deliver_hypothesis(), 
                                   transitions={'failed':'DELIVER HYPOTHESIS',
                                                'succeded':'HYPOTHESIS CHECK'})
        smach.StateMachine.add('HYPOTHESIS CHECK', Hypothesis_check(), 
                                   transitions={'right':'end',
                                                'wrong':'EXPLORE'})


    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    sm_top.execute()

    rospy.spin()
    sis.stop()

    

if __name__ == '__main__':
    main()

