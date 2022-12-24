#!/usr/bin/python3
import tf
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from ddcontroller import DDRobot

def commandVelocity(msg):

    robot.set_motion([msg.linear.x, msg.angular.z])

def updatePose(msg):

    pose = msg.pose.pose
    orientation = pose.orientation
    position = pose.position

    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    robot.define_heading(yaw)
    robot.define_global_position([position.x, position.y])

if __name__=="__main__":

    robot = DDRobot()

    rospy.init_node("scuttle_driver")
    r = rospy.Rate(50) # 50hz

    # Get enable_tf_publish enable_joint_state_publish flags
    enable_tf_publish          = rospy.get_param("/enable_tf_publish", True)
    enable_joint_state_publish = rospy.get_param("/enable_joint_state_publish", True)

    rospy.Subscriber("/cmd_vel", Twist, commandVelocity)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, updatePose)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    if enable_tf_publish is True:
        odom_broadcaster = tf.TransformBroadcaster()

    try:

        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            x, y = robot.get_global_position()
            vx, vth = robot.get_motion()
            th = robot.get_heading()

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # Publish TF when enable_tf_pulish is true
            if enable_tf_publish is True:
                odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
                )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
            # Since some ROS nodes expect non-zero pose covariance,
            # set non-zero default pose covariance matrix.
            odom.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1]

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, 0))

            odom_pub.publish(odom)

            # Publish joint state when enable_joint_state_publish is true
            if enable_joint_state_publish is True:
                myJointStatePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)
                jointState = JointState()

                jointState.header = Header()

                jointState.header.stamp = rospy.Time.now()

                jointState.name = ['l_wheel_joint',
                                   'r_wheel_joint',
                                   'r_caster_swivel_joint',
                                   'l_caster_swivel_joint',
                                   'r_caster_wheel_joint',
                                   'l_caster_wheel_joint'
                                  ]

                jointState.position = [robot.left_wheel.encoder.position * ((2 * np.pi) / robot.left_wheel.encoder.resolution),
                                       robot.right_wheel.encoder.position * ((2 * np.pi) / robot.right_wheel.encoder.resolution),
                                       0,
                                       0,
                                       0,
                                       0
                                      ]

                jointState.velocity = []
                jointState.effort = []
                myJointStatePublisher.publish(jointState)

            r.sleep()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        print('Stopping...')
        # pass

    finally:
        robot.stop()
