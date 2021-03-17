#!/usr/bin/python
import yaml

from gibson.envs.husky_env import HuskyNavigateEnv
import os
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
import rospkg
import numpy as np
from cv_bridge import CvBridge

import tf

MATTERPORT_DATASET = 'matterport'
STANFORD_DATASET = 'stanford'

def callback(data):
    global front_left, front_right, rear_left, rear_right
    front_left = data.linear.x/10.0 - data.angular.z / 50.0
    rear_left = data.linear.x/10.0 - data.angular.z / 50.0
    front_right = data.linear.x/10.0 + data.angular.z / 50.0
    rear_right = data.linear.x/10.0 + data.angular.z / 50.0

def callback_closure(resolution, has_semantics):
    def callback_step(data):
        global front_left, front_right, rear_left, rear_right, bridge
        obs, _, _, _ = env.step([front_left, front_right, rear_left, rear_right])
        rgb = obs['rgb_filled']
        depth = obs['depth'].astype(np.float32)
        depth[depth > 10] = 10
        #depth[depth < 0.45] = np.nan
        rgb_image_message = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        depth_raw_image = (obs["depth"] * 1000).astype(np.uint16)
        depth_raw_message = bridge.cv2_to_imgmsg(depth_raw_image, encoding='passthrough')
        depth_message = bridge.cv2_to_imgmsg(depth, encoding='passthrough')

        now = rospy.Time.now()

        rgb_image_message.header.stamp = now
        depth_message.header.stamp = now
        depth_raw_message.header.stamp = now
        rgb_image_message.header.frame_id='eyes'
        depth_message.header.frame_id='eyes'
        depth_raw_message.header.frame_id='eyes'

        image_pub.publish(rgb_image_message)
        depth_pub.publish(depth_message)
        depth_raw_pub.publish(depth_raw_message)
        msg = CameraInfo(height=resolution, width=resolution, distortion_model="plumb_bob", D=[0.0, 0.0, 0.0, 0.0, 0.0],
                         K=[128, 0.0, 128.5, 0.0, 128, 128.5, 0.0, 0.0, 1.0],
                         R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                         P=[128, 0.0, 128.5, -0.0, 0.0, 128, 128.5, 0.0, 0.0, 0.0, 1.0, 0.0])
        msg.header.stamp = now
        msg.header.frame_id="eyes"
        camera_info_pub.publish(msg)

        # Publish semantic image if it is available
        if has_semantics:
            semantics = obs['semantics']
            semantic_image_message = bridge.cv2_to_imgmsg(semantics.astype(np.uint8), encoding='rgb8')
            semantic_image_message.header.stamp = now
            semantic_image_message.header.frame_id='eyes'
            semantics_pub.publish(semantic_image_message)

        # Odometry
        odom = np.array(env.robot.body_xyz) - np.array(env.config["initial_pos"]), np.array(env.robot.body_rpy)

        br.sendTransform((odom[0][0], odom[0][1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1]),
                         rospy.Time.now(),
                         'base_footprint',
                         'odom')
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = odom[0][0]
        odom_msg.pose.pose.position.y = odom[0][1]
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, \
        odom_msg.pose.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, odom[-1][-1])
        odom_msg.twist.twist.linear.x = (front_right + front_left) * 5
        odom_msg.twist.twist.angular.z = (front_left - front_right) * 25

        odom_pub.publish(odom_msg)

    return callback_step


if __name__ == '__main__':
    rospy.init_node('ros_gibson_environment_simulator')

    # Get parameters
    environment = rospy.get_param(rospy.get_name() + '/environment', default='house1')

    resolution = int(rospy.get_param(rospy.get_name() + '/resolution', default=256))

    # Load gibson config parameter
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ros_gibson_environment')
    with open(os.path.join(package_path, 'config', 'package_config.yaml'), mode='r') as config_file:
        gibson_config = yaml.load(config_file)['gibson_config']

    # Load starting positions
    with open(os.path.join(package_path, 'config', 'starting_positions.yaml'), mode='r') as starting_positions_file:
        starting_positions = yaml.load(starting_positions_file)

    # Get environment data from starting_positions file
    position = starting_positions[environment]['position']
    position[2] += 0.13
    orientation = starting_positions[environment]['orientation']
    dataset = starting_positions[environment]['dataset']
    has_semantics = starting_positions[environment]['semantics']

    # Create gibson config file based of dataset type and its environment
    gibson_config['envname'] = 'HuskyNavigateEnv'
    gibson_config['model_id'] = environment
    gibson_config['initial_pos'] = position
    gibson_config['initial_orn'] = orientation
    gibson_config['resolution'] = resolution

    # Check if the environment has semantic data
    if has_semantics:
        gibson_config['output'] = ['nonviz_sensor', 'rgb_filled', 'depth', 'semantics']
        gibson_config['ui_components'] = ['RGB_FILLED', 'DEPTH', 'SEMANTICS']
        gibson_config['ui_num'] = 3
        if dataset == STANFORD_DATASET:
            gibson_config['semantic_source'] = 1
            gibson_config['semantic_color'] = 3
        elif dataset == STANFORD_DATASET:
            gibson_config['semantic_source'] = 2
            gibson_config['semantic_color'] = 2
    else:
        gibson_config['output'] = ['nonviz_sensor', 'rgb_filled', 'depth']
        gibson_config['ui_components'] = ['RGB_FILLED', 'DEPTH']
        gibson_config['ui_num'] = 2

    # Write gibson config parameter in a file
    gibson_config_file_path = os.path.join(package_path, 'config', 'gibson_config_file.yaml')
    with open(gibson_config_file_path, mode='w') as gibson_config_file:
        yaml.dump(gibson_config, gibson_config_file, default_flow_style=False)

    front_left = 0.0
    front_right = 0.0
    rear_left = 0.0
    rear_right = 0.0
    image_pub = rospy.Publisher('/gibson_ros/camera/rgb/image', Image, queue_size=10)
    depth_pub = rospy.Publisher('/gibson_ros/camera/depth/image', Image, queue_size=10)
    depth_raw_pub = rospy.Publisher('/gibson_ros/camera/depth/image_raw', Image, queue_size=10)
    semantics_pub = rospy.Publisher('/gibson_ros/camera/semantic/image', Image, queue_size=10)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

    camera_info_pub = rospy.Publisher('/gibson_ros/camera/depth/camera_info', CameraInfo, queue_size=10)
    bridge = CvBridge()
    br = tf.TransformBroadcaster()

    env = HuskyNavigateEnv(config=gibson_config_file_path)
    print(env.config)

    obs = env.reset()

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'base_footprint',
                     'odom')

    rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, callback)
    rospy.Subscriber('/gibson_ros/sim_clock', Int64, callback_closure(resolution, has_semantics))

    rospy.spin()





