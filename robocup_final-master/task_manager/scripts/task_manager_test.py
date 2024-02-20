#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool ,Float32

#from task_manager.srv import *

# define states
# Find target on the table
class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['succeeded'])
        self.findtarget_pub = rospy.Publisher('find_target', Bool, queue_size=1)
        self.find_target = Bool()
        self.find_target = True

    def execute(self, userdata):
        rospy.sleep(5.0) #wait camera delay
        self.findtarget_pub.publish(self.find_target)
        rospy.loginfo('looking for a grasping target')
        
        return "succeeded"
#check distance to the place    
class CheckDistance_table(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded'])
        rospy.Subscriber('/distance_to_table', Float32, self.distance_callback)
        self.move_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.distance = 0
        self.move = Twist()
        self.table_data_updated = False
    def distance_callback(self,data):
        self.distance = data.data
        self.table_data_updated = True
    def execute(self, userdata):
        rospy.sleep(4.0)

        rospy.loginfo('distance to table is : %.2f',self.distance)
        while not rospy.is_shutdown():
            if self.table_data_updated:
                self.table_data_updated = False
                if self.distance <=0.8:
                    #go back
                    self.move.linear.x = -0.08
                    #self.move.angular.z = -0.2
                    begin_time = rospy.Time.now()
                    duration = rospy.Duration(0.2)
                    end_time = begin_time + duration
                    while (rospy.Time.now() < end_time):
                        self.move_pub.publish(self.move)
                    return "aborted"
                elif self.distance >=0.85:
                    self.move.linear.x = 0.08
                    #self.move.angular.z = 0.2
                    begin_time = rospy.Time.now()
                    duration = rospy.Duration(0.2)
                    end_time = begin_time + duration
                    while (rospy.Time.now() < end_time):
                        self.move_pub.publish(self.move)
                    return "aborted" 
                else:
                    return "succeeded"

class CheckDistance_shelf(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded'])
        rospy.Subscriber('/distance_to_table', Float32, self.distance_callback)
        self.move_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.distance = 0
        self.move = Twist()
        self.table_data_updated = False
        
    def distance_callback(self,data):
        self.distance = data.data
        self.table_data_updated = True
    def execute(self, userdata):
        rospy.sleep(4.0)
        rospy.loginfo('distance to table is : %.2f',self.distance)
        while not rospy.is_shutdown():
            if self.table_data_updated:
                self.table_data_updated = False
                if self.distance <=0.8:
                    #go back
                    self.move.linear.x = -0.05
                
                    begin_time = rospy.Time.now()
                    duration = rospy.Duration(0.3)
                    end_time = begin_time + duration
                    while (rospy.Time.now() < end_time):
                        self.move_pub.publish(self.move)
                    return "aborted"
                elif self.distance >=0.85:
                    self.move.linear.x = 0.05
                    
                    begin_time = rospy.Time.now()
                    duration = rospy.Duration(0.3)
                    end_time = begin_time + duration
                    while (rospy.Time.now() < end_time):
                        self.move_pub.publish(self.move)
                    return "aborted" 
                else:
                    return "succeeded"

# Find a place to put down the object
class FindPlace(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['succeeded'])
        self.findplace_pub = rospy.Publisher('find_place', Bool, queue_size=1)
        self.find_place = Bool()
        self.find_place = True

    def execute(self, userdata):
        rospy.sleep(5.0)
        self.findplace_pub.publish(self.find_place)
        rospy.loginfo('looking for a place to put down the object')
        

        return "succeeded"
    
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        #self.pub = rospy.Publisher('grasp_object', Bool, queue_size=1)
        """ self.grasp = Bool()
        self.grasp = True"""
        self.hold_object = False #check grasping process done 
        self.gripper_succeed = False # check if grasp any objects
        rospy.Subscriber('/pick_done', Bool, self.hold_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)  # Subscribe to the "joint_state" topic
        self.gripper_succeed_pub = rospy.Publisher('/gripper_succeed' ,Bool, queue_size=1)
    def hold_callback(self,data):
        self.hold_object = data.data
    def joint_state_callback(self,data):
        left_gripper_position = data.position[7]
        right_gripper_position = data.position[8]
        if (left_gripper_position + right_gripper_position) < 0.02:
            self.gripper_succeed = False
        else:
            self.gripper_succeed = True
    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')

        #self.pub.publish(self.grasp)
        while self.hold_object==False:
            rospy.sleep(1.0)
        self.hold_object = False
        print("gripper state is :", self.gripper_succeed)
        if self.gripper_succeed == True:
            self.gripper_succeed_pub.publish(self.gripper_succeed)
            return "succeeded"
        else:
            self.gripper_succeed_pub.publish(self.gripper_succeed)
            rospy.sleep(8.0)   # wait for torso going down
            return "succeeded"
        
class Putdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        """         self.pub_putdown = rospy.Publisher('put_down', Bool, queue_size=1)
        self.put = Bool()
        self.put = True """
        self.put_down = False
        
        rospy.Subscriber('/place_done', Bool, self.place_callback)
    def place_callback(self,data):
        self.put_down = data.data
 
    def execute(self, userdata):
        rospy.loginfo("Executing placing object")
     
        #self.pub_putdown.publish(self.put)
        while self.put_down==False:
            rospy.sleep(1)
        self.put_down = False
        return "succeeded"


# main
def main():
    rospy.init_node('smach_example_state_machine')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    sm.userdata.object_position = 0
    sm.userdata.hold_object = 0
    sm.userdata.num_object = rospy.get_param('num_object')
    # Open the container
    with sm:

        # Navigation callback
        def nav_cb(userdata, goal):
            navGoal = MoveBaseGoal()
            navGoal.target_pose.header.frame_id = "map"
            print("num_object",userdata.num_object)
            if userdata.navGoalInd == 1:
                rospy.loginfo('Navagate to table')
                waypoint = rospy.get_param('/way_points/table_one')
                userdata.navGoalInd = 2
            elif userdata.navGoalInd == 2:
                rospy.loginfo('Navagate to shelf')
                waypoint = rospy.get_param('/way_points/table_two')
                userdata.num_object = userdata.num_object + 1
                if userdata.num_object < 3:
                    userdata.navGoalInd = 1
                else:
                    userdata.navGoalInd = 3
            elif userdata.navGoalInd == 3:
                rospy.loginfo('Navagate to kitchen')
                waypoint = rospy.get_param('/way_points/table_three')
                userdata.navGoalInd = 2
            navGoal.target_pose.pose.position.x = waypoint["x"]
            navGoal.target_pose.pose.position.y = waypoint["y"]
            navGoal.target_pose.pose.orientation.z = waypoint["z"]
            navGoal.target_pose.pose.orientation.w = waypoint["w"]
            navGoal.target_pose.pose.orientation.x = waypoint["wx"]
            navGoal.target_pose.pose.orientation.y = waypoint["wy"]
            navGoal.target_pose.pose.orientation.z = waypoint["wz"]

            return navGoal

        
        # Add states to the container and define the trasitions
        # Navigate to user defined waypoint with callback
        smach.StateMachine.add('NAVIGATION_TO_TABLE', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd','num_object'], output_keys=['navGoalInd','num_object']), 
                                transitions={'succeeded':'IDENTIFY_OBJECT',
                                            'aborted':'aborted'})

        smach.StateMachine.add('NAVIGATION_TO_SHELF', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd','num_object'], output_keys=['navGoalInd','num_object']), 
                                transitions={'succeeded':'IDENTIFY_PLACE',
                                            'aborted':'aborted'},
                                )
        smach.StateMachine.add('NAVIGATION_TO_KITCHEN', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd','num_object'], output_keys=['navGoalInd','num_object']), 
                                transitions={'succeeded':'IDENTIFY_OBJECT',
                                            'aborted':'aborted'})
        """ smach.StateMachine.add('CHECK_DISTANCE_TABLE', CheckDistance_table(), 
                                transitions={'succeeded':'IDENTIFY_OBJECT', 
                                            'aborted':'CHECK_DISTANCE_TABLE'}) 
        smach.StateMachine.add('CHECK_DISTANCE_SHELF', CheckDistance_shelf(), 
                                transitions={'succeeded':'IDENTIFY_PLACE', 
                                            'aborted':'CHECK_DISTANCE_SHELF'}) """
        smach.StateMachine.add('IDENTIFY_OBJECT', FindTargat(), 
                                transitions={'succeeded':'GRASP'})
        smach.StateMachine.add('IDENTIFY_PLACE', FindPlace(), 
                                transitions={'succeeded':'PUT_DOWN'})
        smach.StateMachine.add('GRASP', Grasp(), 
                                transitions={'succeeded':'NAVIGATION_TO_SHELF'})
        smach.StateMachine.add('PUT_DOWN', Putdown(), 
                                transitions={'succeeded':'NAVIGATION_TO_TABLE'})
    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(outcome)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()