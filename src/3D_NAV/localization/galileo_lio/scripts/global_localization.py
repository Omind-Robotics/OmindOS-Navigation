#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations
from std_msgs.msg import String
import json
import logging
import os


global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None

# #######################################################
# new 2025.08.01 liu 
initial2DPose = None
get2DPose = None  # 默认值 = 0； 收到rviz传来的重定位功能 = 1；收到平台或者狗从充电房出来的重定位时 = 2；
json_type = None
json_pose_x = None
json_pose_y = None
json_pose_z = None
json_ori_x = None
json_ori_y = None
json_ori_z = None
json_ori_w = None
logged_waiting_message = False

# 下面是python将rospy.loginfo和rospy.logwarn等函数的输出重定向到日志文件的方法 20250826 new liu
# # kong110
# log_dir = '/home/kong110/jll_pro/files/logs/galileo_localization'

# slam
# log_dir = '/home/slam/jll_pro/files/logs/galileo_localization'
log_dir = '/home/arts/Galilio/3D_nav_ws/src/localization/galileo_lio/logs'

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# 带颜色的日志格式化器
class ColoredFormatter(logging.Formatter):
    """给不同级别的日志添加颜色"""
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format_str = "%(asctime)s - %(levelname)s - %(message)s"

    def __init__(self):
        super().__init__()
        self.fmt = {
            logging.DEBUG: self.grey + self.format_str + self.reset,
            logging.INFO: self.grey + self.format_str + self.reset,
            logging.WARNING: self.yellow + self.format_str + self.reset,
            logging.ERROR: self.red + self.format_str + self.reset,
            logging.CRITICAL: self.bold_red + self.format_str + self.reset
        }

    def format(self, record):
        log_fmt = self.fmt.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt='%Y-%m-%d %H:%M:%S')
        return formatter.format(record)
    

# 创建自定义日志记录器
def setup_logging():
    from datetime import datetime
    
    # 生成带时间戳的日志文件名
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f'{current_time}.log')
    
    # 创建日志记录器
    logger = logging.getLogger('galileo_localization')
    logger.setLevel(logging.DEBUG)
    
    # 清除所有现有处理器
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # 文件处理器配置
    file_handler = logging.FileHandler(log_file, mode='a')
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_formatter)
    
    # 控制台处理器配置(带颜色)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(ColoredFormatter())
    
    # 添加处理器
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    # 重定向ROSPY日志
    def rospy_log_to_python_log(msg, *args, **kwargs):
        level = kwargs.get('level', logging.INFO)
        logger.log(level, msg, *args)
    
    rospy.loginfo = lambda msg, *args: rospy_log_to_python_log(msg, *args, level=logging.INFO)
    rospy.logwarn = lambda msg, *args: rospy_log_to_python_log(msg, *args, level=logging.WARNING)
    rospy.logerr = lambda msg, *args: rospy_log_to_python_log(msg, *args, level=logging.ERROR)
    rospy.logdebug = lambda msg, *args: rospy_log_to_python_log(msg, *args, level=logging.DEBUG)
    
    return logger

# 初始化日志系统
logger = setup_logging()
logger.info('Logging system initialized successfully.')

# #######################################################

def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )

    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    logger.info('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    logger.info('Time: {}'.format(toc - tic))
    logger.info('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        logger.warning('Not match!!!!')
        logger.warning('{}'.format(transformation))
        logger.warning('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    logger.info('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


# 订阅重定位消息 new 2025.07.28 liu
def relocation_callback(msg: String):
    """
    回调函数：将收到的字符串解析为 JSON 并打印结果
    """
    try:
        data = json.loads(msg.data)  # 解析 JSON
        # logger.info("收到 JSON: %s", data)

        # ====== 这里可以按自己的 JSON 字段做业务处理 ======
        global json_type, json_pose_x, json_pose_y, json_pose_z, json_ori_x, json_ori_y, json_ori_z, json_ori_w, get2DPose
        get2DPose = 2
        json_type = data.get("type")
        json_pose_x = data.get("data").get("pos_x")
        json_pose_y = data.get("data").get("pos_y")
        json_pose_z = data.get("data").get("pos_z")
        json_ori_x = data.get("data").get("ori_x")
        json_ori_y = data.get("data").get("ori_y")
        json_ori_z = data.get("data").get("ori_z")
        json_ori_w = data.get("data").get("ori_w")
        logger.info("json_type = %s", json_type)
        logger.info("json_pose_x = %s", json_pose_x)
        logger.info("json_pose_y = %s", json_pose_y)
        logger.info("json_pose_z = %s", json_pose_z)
        logger.info("json_ori_x = %s", json_ori_x)
        logger.info("json_ori_y = %s", json_ori_y)
        logger.info("json_ori_z = %s", json_ori_z)
        logger.info("json_ori_w = %s", json_ori_w)
        # ==================================================

        # test 2025.08.01 liu
        # data = {"type": json_type, "data": {"relocation": 0}}
        # data_str = json.dumps(data)  # 将字典转换为 JSON 格式的字符串
        # while pub_relocation_.get_num_connections() > 0:
        #     # 发布数据
        #     logger.info("Publishing data: %s", data_str)
        #     pub_relocation_.publish(data_str)
        #     break
        
        # 触发全局定位
        trigger_global_localization()

    except json.JSONDecodeError as e:
        logger.error("JSON 解析失败：%s", e)


# 订阅重定位消息 new 2025.07.28 liu
def initial_pose_callback(msg: PoseWithCovarianceStamped):
    try:
        global initial2DPose, get2DPose
        get2DPose = 1
        initial2DPose = msg
        logger.info("!!! 从Rviz处获取初始位姿 !!!")

        # 触发全局定位
        trigger_global_localization()
    except json.JSONDecodeError as e:
        logger.error("获取初始位姿失败: %s", e)


def trigger_global_localization():
    global get2DPose, initial2DPose, cur_scan, T_map_to_odom, logged_waiting_message, json_type
    if get2DPose == 1 and initial2DPose and cur_scan:
        initial_pose = pose_to_mat(initial2DPose)
        initialized = global_localization(initial_pose)
        if initialized == True:
            # new 2025.08.01 liu
            logged_waiting_message = True
            logger.info('!!! Rviz下发的初始位姿成功匹配到全局地图, 重定位成功 !!!')
            data = {"type": 1, "data": {"relocation": 1}}
            data_str = json.dumps(data)  # 将字典转换为 JSON 格式的字符串
            
            # 发布数据
            logger.info("Publishing data: %s", data_str)
            pub_relocation_.publish(data_str)
        else:
            # new 2025.08.01 liu
            logged_waiting_message = False
            logger.error('!!! Rviz下发的初始位姿没有匹配到全局地图, 重定位失败 !!!')
            data = {"type": 1, "data": {"relocation": 0}}
            data_str = json.dumps(data)  # 将字典转换为 JSON 格式的字符串
            
            # 发布数据
            logger.info("Publishing data: %s", data_str)
            pub_relocation_.publish(data_str)
        get2DPose = 0
    elif get2DPose == 2 and cur_scan:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position.x = json_pose_x
        pose_msg.pose.pose.position.y = json_pose_y
        pose_msg.pose.pose.position.z = json_pose_z
        pose_msg.pose.pose.orientation.x = json_ori_x
        pose_msg.pose.pose.orientation.y = json_ori_y
        pose_msg.pose.pose.orientation.z = json_ori_z
        pose_msg.pose.pose.orientation.w = json_ori_w
        logger.info('Relocation Initial pose received.')
        initial_pose = pose_to_mat(pose_msg)
        initialized = global_localization(initial_pose)
        print("111 111 111 json type = ", json_type)
        
        if initialized == True:
            # new 2025.08.01 liu
            logged_waiting_message = True
            logger.info('!!! 平台或狗从充电房出来时下发的初始位姿成功匹配到全局地图, 重定位成功 !!!')
            data = {"type": json_type, "data": {"relocation": 1}}
            data_str = json.dumps(data)  # 将字典转换为 JSON 格式的字符串
            
            # 发布数据
            logger.info("Publishing data: %s", data_str)
            pub_relocation_.publish(data_str)
        else:
            # new 2025.08.01 liu
            logged_waiting_message = False
            logger.error('!!! 平台或狗从充电房出来时下发的初始位姿没有匹配到全局地图, 重定位失败 !!!')
            data = {"type": json_type, "data": {"relocation": 0}}
            data_str = json.dumps(data)  # 将字典转换为 JSON 格式的字符串
            
            # 发布数据
            logger.info("Publishing data: %s", data_str)
            pub_relocation_.publish(data_str)
        get2DPose = 0


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.95

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28

    # The farthest distance(meters) within FOV
    FOV_FAR = 30

    rospy.init_node('fast_lio_localization')
    logger.info('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)
    
    # 发布重定位结果
    pub_relocation_ = rospy.Publisher('/slam/relocation_result', String, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 订阅重定位消息 new 2025.07.28 liu
    rospy.Subscriber('/slam/relocation', String, relocation_callback, queue_size=1)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initial_pose_callback, queue_size=1)

    # 初始化全局地图
    logger.warning('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not logged_waiting_message and not rospy.is_shutdown():
        logger.warning('Waiting for initial pose....')
        rospy.sleep(0.5)

    logger.info('')
    logger.info('!!! Initialize successfully !!!')
    logger.info('')
    
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())
    rospy.spin()



