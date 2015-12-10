from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from std_msgs.msg import String, Bool
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Navigator(object):

    def __init__(self):
        rospy.init_node('navigator')

        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        listener = tf.TransformListener()
        listener.waitForTransform('/map', 'base_link', rospy.Time(), rospy.Duration(3.0))

        # Find the initial start position of the robot in terms of the map frame
        (initial_trans, initial_rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
        self.init_x, self.init_y, self.init_th = convert_pose_to_xy_and_theta(convert_translation_rotation_to_pose(initial_trans, initial_rot))

def publishGoal(self, pos_tup=(0.0,0.0,0.0), th=0.0):
        """ Converts the way point position tuple and theta angle in terms
        of the map frame, and publishes a Pose to the ROS goal navigation topic
        pos_tup: a tuple of x, y, z position values
        th: theta or yaw of the robot
        """
        x, y, z = pos_tup
        print("Publishing goal at ({},{},{},{})".format(x,y,z,th))

        # compute final x, y, theta in terms of the map
        final_x = (
            self.init_x +
            x * np.cos(self.init_th) +
            y * np.cos(self.init_th + np.pi / 2)
        )
        final_y = (
            self.init_y +
            x * np.sin(self.init_th) +
            y * np.sin(self.init_th + np.pi / 2)
        )
        final_th = self.init_th + th

        point_msg = Point(final_x, final_y, z)
        quat_msg = Quaternion(*quaternion_from_euler(0,0,final_th))
        pose_msg = Pose(position=point_msg, orientation=quat_msg)

        # in terms of the map frame!
        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id='map')

        pose_stamped = PoseStamped(header=header_msg, pose=pose_msg)
        self.pub.publish(pose_stamped)
