#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import random
from std_msgs.msg import Int32

# define state EXPLORE
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_hint','found_hint'])
        self.rand = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state EXPLORE')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'no_hint'
        else:
            return 'found_hint'
            

# define state UPDATE KNOWLEDGE
class Update_knowledge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','succeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UPDATE KNOWLEDGE')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'failed'
        else:
            return 'succeded'
        
        
# define state CHECK CONSISTENCY
class Check_consistency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['consistent','inconsistent'])
        self.rand = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CHECK CONSISTENCY')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'consistent'
        else:
            return 'inconsistent'
            

# define state REACH ORACLE
class Reach_oracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','reached'])
        self.rand = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state REACH ORACLE')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'failed'
        else:
            return 'reached'
        
        
# define state DELIVER HYPOTHESIS
class Deliver_hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed','succeded'])
        self.rand = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DELIVER HYPOTHESIS')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'failed'
        else:
            return 'succeded'
        

# define state HYPOTHESIS CHECK 
class Hypothesis_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right','wrong'])
        self.rand = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state HYPOTHESIS CHECK ')
        rospy.sleep(3)
        self.rand = random.randint(0,1)
        if self.rand == 0:
            return 'right'
        else:
            return 'wrong'


def main():
    rospy.init_node('cluedo_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['end'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('EXPLORE', Explore(),
                               transitions={'found_hint':'MAKE HYPOTHESIS',
                                            'no_hint':'EXPLORE'})

        # Create the sub SMACH state machine
        mkhy_sub = smach.StateMachine(outcomes=['out1','out2'])

        
        # Open the container
        with mkhy_sub:

            # Add states to the container
            smach.StateMachine.add('UPDATE KNOWLEDGE', Update_knowledge(), 
                                   transitions={'succeded':'CHECK CONSISTENCY', 
                                                'failed':'UPDATE KNOWLEDGE'})
            smach.StateMachine.add('CHECK CONSISTENCY', Check_consistency(), 
                                   transitions={'consistent':'out1', 
                                                'inconsistent':'out2'})

        smach.StateMachine.add('MAKE HYPOTHESIS', mkhy_sub,
                               transitions={'out2':'EXPLORE',
                                            'out1':'GO TO ORACLE'})
        
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

