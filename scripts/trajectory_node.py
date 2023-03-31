import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def perform_trajectory():
    rospy.init_node('trajectory_publisher')
    controller_name = 'arm_trajectory_controller/command'
    trajectory_publisher = rospy.Publisher(controller_name, JointTrajectory, queue_size = 10)
    argv = sys.argv[1:]
    joints = ['fixture_to_base_link', 'base_link_to_upper_leg', 'upper_leg_to_lower_leg']
    goal_positions = [float(argv[0]), float(argv[1]), float(argv[2])]
   
    rospy.loginfo('Goal position set, starting execution.')
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for joint in joints]
    trajectory_msg.points[0].accelerations = [0.0 for joint in joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publisher.publish(trajectory_msg)

if __name__ == '__main__':
    perform_trajectory()    
