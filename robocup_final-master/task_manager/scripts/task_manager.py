#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


# define states
class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted','succeeded','putdown'],input_keys=['in_hold_object'],output_keys=['out_object_position'])
        self.target_object = 2 # target is traffic light
        self.centroids= []
        self.label = []
        #self.dic = ["unknown","sports ball","bottle","cup","banana","apple","bowl","traffic light"]
        rospy.Subscriber('/labeled_objects', PointCloud2, self.pointcloud_callback)
        
    # replace this part by your method
    def pointcloud_callback(self,data):
        pc_data = pc2.read_points(data)
        self.centroids= []
        self.label = []
        for p in pc_data:
            self.centroids.append(p[0:3])
            self.label.append(p[3])
    def execute(self, userdata):
        if userdata.in_hold_object == 0:
            rospy.loginfo('Executing state FindTargat')
            rospy.sleep(5) 
            rospy.loginfo(self.target_object)
            rospy.loginfo(self.label)
            if self.target_object in self.label:
                index = self.label.index(self.target_object)
                userdata.out_object_position = self.centroids[index]
                rospy.loginfo('Target found')
                return 'succeeded'
            else:
                rospy.loginfo('Target not found')
                return 'aborted'
        elif userdata.in_hold_object ==1:
            return 'putdown'
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goto1','goto2'],input_keys=['in_object_position','in_navGoalInd'],output_keys=['out_hold_object'])
        self.pub = rospy.Publisher('/obj_pose', PoseStamped, queue_size=1)
        self.hold_object = False
        rospy.Subscriber('/pick_done', Bool, self.hold_callback)
    def hold_callback(self,data):
        self.hold_object = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state Grasp')
        Object = PoseStamped()
        Object.header.frame_id = "base_footprint"
        Object.pose.position.x = userdata.in_object_position[0]
        Object.pose.position.y = userdata.in_object_position[1]
        Object.pose.position.z = userdata.in_object_position[2]
        self.pub.publish(Object)
        while self.hold_object==False:
            rospy.sleep(1)
        userdata.out_hold_object = 1
        if userdata.in_navGoalInd ==1:
            return 'goto1'
        else:
            return 'goto2'

class Putdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub_putdown = rospy.Publisher('place_pose', PoseStamped, queue_size=1)       


    def execute(self, userdata):
        put = PoseStamped()
        put.pose.position.x = 0.2
        put.pose.position.y = 0.2
        put.pose.position.z = 0.2
        
        self.pub_putdown.publish(put)
        return 'succeeded'
# main
def main():
    rospy.init_node('smach_example_state_machine')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    sm.userdata.object_position = 0
    sm.userdata.hold_object = 0
    # Open the container
    with sm:

        # Navigation callback
        def nav_cb(userdata, goal):
            navGoal = MoveBaseGoal()
            navGoal.target_pose.header.frame_id = "map"
            if userdata.navGoalInd == 1:
                rospy.loginfo('Navagate to table one')
                waypoint = rospy.get_param('/way_points/table_one')
                userdata.navGoalInd = 2
            elif userdata.navGoalInd == 2:
                rospy.loginfo('Navagate to table two')
                waypoint = rospy.get_param('/way_points/table_two')
                userdata.navGoalInd = 1
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
        smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_ONE',
                                            'aborted':'aborted'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO', smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_TWO',
                                            'aborted':'aborted'},
                                )

        smach.StateMachine.add('FIND_TARGET_ON_TABLE_ONE', FindTargat(), 
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_TWO',
                                             'putdown':'PUT_DOWN'},
                                remapping={'out_object_position':'object_position',
                                           'in_hold_object':'hold_object'})
        smach.StateMachine.add('FIND_TARGET_ON_TABLE_TWO', FindTargat(), 
                                transitions={'succeeded':'GRASP', 
                                            'aborted':'NAVIGATION_TO_TABLE_ONE',
                                            'putdown':'PUT_DOWN'},
                                remapping={'out_object_position':'object_position',
                                           'in_hold_object':'hold_object'})
        smach.StateMachine.add('GRASP', Grasp(), 
                                transitions={'goto1':'NAVIGATION_TO_TABLE_ONE',
                                             'goto2':'NAVIGATION_TO_TABLE_TWO'},
                                remapping={'in_object_position':'object_position',
                                           'out_hold_object':'hold_object',
                                           'in_navGoalInd':'navGoalInd'})
        smach.StateMachine.add('PUT_DOWN', Putdown(), 
                                transitions={'succeeded':'preempted'})
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