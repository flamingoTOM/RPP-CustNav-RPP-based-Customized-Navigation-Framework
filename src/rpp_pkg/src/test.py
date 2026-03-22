# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# 订阅 /odom 话题，根据其中的 pose 信息实时广播
# map → base_link 的 TF 变换。
# """

# import rospy
# import tf2_ros
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped


# class OdomToMapTF(object):
#     def __init__(self):
#         rospy.init_node("odom_to_map_tf", anonymous=True)

#         # TF 相关
#         self.tf_br = tf2_ros.TransformBroadcaster()

#         # 订阅 /odom
#         rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

#         rospy.loginfo("odom_to_map_tf node started, broadcasting map -> base_link ...")

#     def odom_cb(self, msg: Odometry):
#         """
#         将 odom 话题里的 pose 直接作为 map -> base_link 的变换发布。
#         注意：此处假设 /odom 给出的就是 map 系下的位姿；
#               如果实际使用的是 odom -> base_link，请根据需求修改。
#         """
#         t = TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "map"
#         t.child_frame_id = "base_test_link"

#         t.transform.translation.x = msg.pose.pose.position.x
#         t.transform.translation.y = msg.pose.pose.position.y
#         t.transform.translation.z = msg.pose.pose.position.z

#         t.transform.rotation = msg.pose.pose.orientation

#         self.tf_br.sendTransform(t)


# if __name__ == "__main__":
#     try:
#         OdomToMapTF()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass