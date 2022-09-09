#!/usr/bin/env python3
'''
smach viewer:
    rosrun smach_viewer smach_viewer.py
    
armore server:
    rosrun armor execute it.emarolab.armor.ARMORMainService
'''
import rospy
import smach
import smach_ros
import random
from std_msgs.msg import Int32

from armor_msgs.msg import * 
from armor_msgs.srv import * 
from cluedo.srv import Hypothesis, Discard, Hints, Compare
from os.path import dirname, realpath

#place = ''
#person = ''
#weapon = ''
#hp = [person,weapon,place]
people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]

srv_client_query_source_ = rospy.ServiceProxy('/query_source', Compare)
srv_client_final_hypothesis_ = rospy.ServiceProxy('/verify_solution', Compare)
srv_client_generate_murder_ = rospy.ServiceProxy('/generate_murder', Hypothesis)  
srv_client_get_hint_ = rospy.ServiceProxy('/get_hint', Hints)

n_hyp = 0

#------------------------ARMOR COMMUNICATION----------------------------------
class Armor_communication():
    ##
    #\class Armor_Communication
    def __init__(self):
        ##
        #\brief Initilize the class by declaring the client to "armor_interface_srv" service
        super(Armor_communication, self).__init__()
        rospy.wait_for_service('armor_interface_srv')
        self.armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)


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
            
    def log_to_file(self):
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
            

        
#    def make_hypothesis(self,person,weapon,place,hyp_code):
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'ADD'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['who',hyp_code,person]
#            msg = self.armor_service(req)
#            
#        except rospy.ServiceException as e:
#            print(e)
#            
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'ADD'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['what',hyp_code,weapon]
#            msg = self.armor_service(req)
#            
#        except rospy.ServiceException as e:
#            print(e)
#
#
#        try:
#            req=ArmorDirectiveReq()
#            req.client_name= 'cluedo'
#            req.reference_name= 'ontology'
#            req.command= 'ADD'
#            req.primary_command_spec= 'OBJECTPROP'
#            req.secondary_command_spec= 'IND'
#            req.args= ['where',hyp_code,place]
#            msg = self.armor_service(req)
#            
#        except rospy.ServiceException as e:
#            print(e)
        
        
    def make_hypothesis(self,hyp,hypID):
        ##
        #\brief Insert an hypothesis in the system
        #@param hypothesis the hypothesis that will be inserted in the system, which is of type hypothesis
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
        print('done')
                
                
                

        
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
            

armor = Armor_communication()

#-------------------------------- State machine classes---------------------------------
'''
# define state INITIALIZATION
Establish the communication with armor server
Loads OWL file
Generates murder to start the game
'''
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')
        armor.__init__()
        armor.log_to_file()
        armor.load_file()
        rospy.wait_for_service("generate_murder") 
        srv_client_generate_murder_()
        rospy.loginfo('Ready')
        return 'ready'
    
'''
# define state EXPLORE
Retrieves the list of available places
Randomly choose one
Reaches place
'''
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_source','found_source'],
                                   input_keys=['place'],
                                   output_keys=['place'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EXPLORE')
        pl_list = armor.retrieve_class('PLACE')
        if pl_list:
            userdata.place = random.choice(pl_list)
            print('I am going to the',userdata.place)
            
        rospy.sleep(3)  # Reach place

        return 'found_source'
            

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
        smach.State.__init__(self, outcomes=['consistent','inconsistent','final hypothesis'],
                                    input_keys=['person','weapon','place'],
                                    output_keys=['person','weapon','place'])

    def execute(self, userdata):
        global n_hyp
        rospy.loginfo('Executing state MAKE HYPOTHESIS')
        
        #debug
#        if armor.retrieve_class('COMPLETED'):
#            for i in range(len(armor.retrieve_class('COMPLETED'))):
#                print(armor.details_of_an_hold_hypothesis(armor.retrieve_class('COMPLETED')[i]))
                
        hyp = srv_client_get_hint_()
        
        hyp_list = [hyp.hint0,hyp.hint1,hyp.hint2,hyp.hint3]
        print(hyp_list)
#        
        
        hypothesis_code = 'HP'+str(n_hyp)
        print(hypothesis_code)
        armor.make_hypothesis(hyp_list, hypothesis_code)
        print('made hp')
        armor.reason()
        print('reasoned')
        compl = armor.retrieve_class('COMPLETED')
        print(compl)
        incons = armor.retrieve_class('INCONSISTENT')
        print(incons)
        
#        if len(compl)==0 and len(incons)==0:
#            print('Armor is not responding...quit program and restart')
#            quit()
        n_hyp = n_hyp +1
        rospy.sleep(3)

        if hypothesis_code in compl and hypothesis_code not in incons: 
            userdata.person = hyp.hint0
            userdata.weapon = hyp.hint1
            userdata.place = hyp.hint2
            return 'consistent' 
        else:
            return 'inconsistent'
        
        
'''
# define state QUERY SOURCE
Send signal to appropriate source
Gets hint through message
Compare hypothesis
Delete hypothesis
Delete common hints
'''
class Query_source(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','succeded'],
                                   input_keys=['person','weapon','place'])

    def execute(self, userdata):
        rospy.loginfo('Executing state QUERY SOURCE')
        print('Hypothesis: It was',userdata.person,'with the',userdata.weapon,'in the',userdata.place)
        
        rospy.sleep(3)
        
        resp = srv_client_final_hypothesis_(userdata.person, userdata.weapon, userdata.place)
        if resp.person and resp.weapon and resp.place is True:
        
            return 'succeded'
        else:
            return 'failed'
        
        
    

# define state REACH ORACLE
class Reach_oracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','reached'])

    def execute(self, userdata):
        rospy.loginfo('Executing state REACH ORACLE')
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
        rospy.loginfo('Executing state DELIVER HYPOTHESIS')
        
        if userdata.person and userdata.weapon and userdata.place:
            print('Hypothesis: It was',userdata.person,'with the',userdata.weapon,'in the',userdata.place)
            return 'succeded'
        else:
            print('The hypothesis has failed to be delivered')
            return 'failed'
        

# define state HYPOTHESIS CHECK 
class Hypothesis_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right','wrong'],
                             input_keys=['person','weapon','place'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HYPOTHESIS CHECK ')
        rospy.sleep(3)
        resp = srv_client_final_hypothesis_(userdata.person, userdata.weapon, userdata.place)
        
        if resp.person and resp.weapon and resp.place is True:
            return 'right'
        else:
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
                               transitions={'ready':'EXPLORE'})
    
        smach.StateMachine.add('EXPLORE', Explore(),
                               transitions={'found_source':'MAKE HYPOTHESIS',
                                            'no_source':'EXPLORE'})
        
        smach.StateMachine.add('MAKE HYPOTHESIS', Make_hypothesis(),
                               transitions={'consistent':'QUERY SOURCE',
                                            'inconsistent':'MAKE HYPOTHESIS',
                                            'final hypothesis':'GO TO ORACLE'})
        
        smach.StateMachine.add('QUERY SOURCE', Query_source(),
                               transitions={'succeded':'GO TO ORACLE',
                                            'failed':'EXPLORE'})

        # Create the sub SMACH state machine
#        mkhy_sub = smach.StateMachine(outcomes=['out1','out2'])

        
#        # Open the container
#        with mkhy_sub:
#
#            # Add states to the container
#            smach.StateMachine.add('UPDATE KNOWLEDGE', Update_knowledge(), 
#                                   transitions={'succeded':'CHECK CONSISTENCY', 
#                                                'failed':'UPDATE KNOWLEDGE'})
#            smach.StateMachine.add('CHECK CONSISTENCY', Check_consistency(), 
#                                   transitions={'consistent':'out1', 
#                                                'inconsistent':'out2'})
#
#        smach.StateMachine.add('MAKE HYPOTHESIS', mkhy_sub,
#                               transitions={'out2':'EXPLORE',
#                                            'out1':'GO TO ORACLE'})
        
        # Create the sub SMACH state machine
        gtor_sub = smach.StateMachine(outcomes=['out3','out4'])

        # Open the container
        with gtor_sub:

            # Add states to the container
            smach.StateMachine.add('REACH ORACLE', Reach_oracle(), 
                                   transitions={'failed':'REACH ORACLE', 
                                                'reached':'DELIVER HYPOTHESIS'})
            smach.StateMachine.add('DELIVER HYPOTHESIS', Deliver_hypothesis(), 
                                   transitions={'failed':'DELIVER HYPOTHESIS',
                                                'succeded':'HYPOTHESIS CHECK'})
            smach.StateMachine.add('HYPOTHESIS CHECK', Hypothesis_check(), 
                                   transitions={'right':'out3',
                                                'wrong':'out4'})

        smach.StateMachine.add('GO TO ORACLE', gtor_sub,
                               transitions={'out3':'end',
                                            'out4':'EXPLORE'})

    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

    

if __name__ == '__main__':
    main()

