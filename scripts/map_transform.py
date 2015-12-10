#!/usr/bin/env python

import rospy
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

rospy.init_node('map_transform')
pub = rospy.Publisher('position', PoseStamped, queue_size=10)
listener = TransformListener()
r = rospy.Rate(5)

while not rospy.is_shutdown():
	try:
		now = rospy.Time.now()
		listener.waitForTransform('map','base_link',now,rospy.Duration(0.5))
		pose_out = listener.transformPose("map",
							   PoseStamped(header=Header(stamp=now,
											 frame_id="base_link"),
							   			   pose=Pose()))
		pub.publish(pose_out)
	except Exception as inst:
		print inst
	r.sleep()