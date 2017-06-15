#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import math
import datetime
import time
import numpy as np
import scipy
import cv2
import os
import csv
import tf
import re
import hsrb_interface

from hsrb_interface import Robot
from hsrb_interface import geometry
from hsrb_interface import ItemTypes
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray, String

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)

_CONNECTION_TIMEOUT = 10.0

# Create speak sentences
# Index 0: in Japanese, 1: in English
_EXPLAIN1 = [u'僕はものを掴むだけで重さがわかっちゃうんだ。すごいでしょ。グリッパの間に重さをはかりたいものを持ってきてください',
             u'Please set the object between my gripper']
_EXPLAIN2 = [u'グリッパを閉じます', u'I close my hand now']
_ANSWER = [u'これは{0}グラムです', u'This is {0} gram']


_INITIAL_POS = [u'自己位置を地図上の位置に一致させます。', u'']
_INITIAL_X = [u'自己位置のエックス座標は{0}ですね', u'initial x is {0}']
_INITIAL_Y = [u'自己位置のワイ座標は{0}ですね', u'initial y is {0}']
_INITIAL_ANGLE = [u'自己位置の姿勢は{0}ですね', u'initial theta is {0}']

_STANDBY = [u'イレイサー、準備完了。', u'eraser standby']
_START = [u'ロボット工房の案内プログラムを起動しました。ドアを開けてください。', u'']
_OPENDOOR = [u'ドアがあきました。部屋に入ります。', u'']
_GREETING1 = [u'こんにちは、僕はイレイサーだよ。これからロボット工房を案内するね。', u'Hello, My name is eraser']
_GREETING2 = [u'みなさん改めてこんにちは。僕は家庭でみなさんの手伝いをするために作られたロボットなんだ。飲み物を取ってきたり、床に落ちたゴミを拾ったりできるんだよ。まずは、ここロボット工房の案内をするよ。', u'Hello, My name is eraser']
_GREETING3 = [u'僕はロボカップの世界大会で優勝したこともあるんだ、すごいでしょ。', u'Hello, My name is eraser']
_REFRIGERATOR =[u'ここは冷蔵庫だよ。暑い夏には冷たい飲み物が欠かせないよね', u'']
_BOOKSHELF = [ u'この本棚には学生さんたちの教科書がはいっているよ。学生は勉強が一番だからね。', u'']
_SHELF1 = [u'さて、これから僕のすごいところを見せちゃうよ。棚にある飲み物を取って、届けるよ。何がいいかなぁ、じゃあバナバオーレにしよう。僕はバナナオーレが好きなんだ。', u'']
_SHELF2 = [u'バナナオーレを見つけました。', u'']
_NG1 = [u'残念、失敗しました。バナナオーレを掴めませんでした。', u'fail to grasp']
_BY = [u'皆さん、今日はロボット工房まで来てくれてありがとう。またお会いしましょう。質問は岡田先生にしてね', u'Good by']

def compute_difference(pre_data_list, post_data_list):
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
    # Calcurate square sum of difference
    square_sums = sum([math.pow(b - a, 2)
                       for (a, b) in zip(pre_data_list, post_data_list)])
    return math.sqrt(square_sums)


class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=_CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z


class JointController(object):
    """Control arm and gripper"""

    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(
            joint_control_service, SafeJointChange)

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(
                timeout=_CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(
                    _CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def move_to_joint_positions(self, goal_joint_states):
        """Joint position control"""
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success

    def grasp(self, effort):
        """Gripper torque control"""
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


class Speaker(object):
    """Speak sentence in robot's language"""

    def __init__(self):
        talk_action = '/talk_request_action'
        self._talk_request_client = actionlib.SimpleActionClient(
            talk_action, TalkRequestAction)

        # Wait for connection
        try:
            if not self._talk_request_client.wait_for_server(
                    rospy.Duration(_CONNECTION_TIMEOUT)):
                raise Exception(talk_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # Detect robot's language
        self._lang = Voice.kJapanese
#        if os.environ['LANG'] == 'ja_JP.UTF-8':
#            self._lang = Voice.kJapanese
#        else:
#            self._lang = Voice.kEnglish

    def get_language(self):
        return self._lang

    def speak_sentence(self, sentence):
        goal = TalkRequestGoal()
        goal.data.language = self._lang
        goal.data.sentence = sentence

        if (self._talk_request_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持トルク[Nm]
_GRASP_TORQUE=-0.01
# ボトルのtf名
_BOTTLE_TF='ar_marker/1'
# グリッパのtf名
_HAND_TF='hand_palm_link'


# ロボット機能を使うための準備
robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')
gripper = robot.get('gripper')


# bottleのマーカーの手前0.02[m],z軸回に-1.57回転させた姿勢
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# handを0.1[m]上に移動させる姿勢
hand_up = geometry.pose(x=0.1)

# handを0.5[m]手前に移動させる姿勢
hand_back = geometry.pose(z=-0.5)


### 場所の座標
# ROSは右手座標系standby_pos = (-1.94, 0.0, 0) # 待機場所の座標
zero_pos = (0, 0.0, 0.0) # 原点
init_greeting_pos = (0.5, 0.0, 1.57) # 見学者に挨拶する場所
refrigerator_pos = (-0.4, -1.1, -1.57) # 冷蔵庫
bookshelf_pos = (4.21, -1.35, -1.57) # 本棚の場所
shelf_front_pos = (1.14, 1.65, 3.14) # 食器棚の見える場所
shelf_pos = (-0.3, 0.92, 3.14) # 食器棚の場所
paper_pos = (2.8, 1.4, 0) # 紙が落ちている場所
greeting_pos = (1.14, 1.65, 1.57) # 見学者に挨拶する場所



if __name__=='__main__':
    speaker = Speaker()

    #############
#     tts.say(u'ロボット工房の案内を中止します。')
#    rospy.sleep(3)
#    base.go(standby_pos[0], standby_pos[1], standby_pos[2], 180.0)
#    sys.exit()
    #############
    #############


    # 初期姿勢に遷移
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')

    # 自己位置（ドア前）を地図上の位置に一致させる
    speaker.speak_sentence(_INITIAL_POS[speaker.get_language()])
    rospy.sleep(2)

#    init_x = rospy.get_param("~init_x", 0.0)
#    init_y = rospy.get_param("~init_y", 0.0)
#    init_angle = rospy.get_param("~init_angle", 0.0)    
    init_x = standby_pos[0]
    init_y = standby_pos[1]
    init_angle = standby_pos[2]

    speaker.speak_sentence(_INITIAL_X[speaker.get_language()].format(init_x))
    speaker.speak_sentence(_INITIAL_Y[speaker.get_language()].format(init_y))
    speaker.speak_sentence(_INITIAL_ANGLE[speaker.get_language()].format(init_angle))

    point = Point(init_x, init_y, 0.0) #x座標とy座標、z座標は無視される
    angle = init_angle #姿勢をラジアンで与える
    rospy.loginfo("setRealPosition:x=%f, y=%f, th=%f",init_x, init_y, init_angle)
    pub = rospy.Publisher('/laser_2d_correct_pose',
                          PoseWithCovarianceStamped,
                          latch=True, queue_size=10)
    p   = PoseWithCovarianceStamped();
    msg = PoseWithCovariance();
    q_angle = quaternion_from_euler(0, 0, angle, 'sxyz')
    q = Quaternion(*q_angle)
    msg.pose = Pose(point, q);
    msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853];
    p.pose = msg;
    p.header.stamp = rospy.Time.now()
    p.header.frame_id="map"
    pub.publish(p);
    rospy.sleep(3)


    # ドアの前でスタンバイ
    speaker.speak_sentence(_STANDBY[speaker.get_language()])

    # 腕を押したらタスクを開始する
    while True:
        wrench = robot.get('wrist_wrench',ItemTypes.FORCE_TORQUE).wrench
        if wrench[0][0] > 20.0:
            break
        pass

    speaker.speak_sentence(_START[speaker.get_language()])
    rospy.sleep(2)

    # ドアが開いたら部屋に入る
    speaker.speak_sentence(_OPENDOOR[speaker.get_language()])


    # 原点に移動
    base.go(zero_pos[0], zero_pos[1], zero_pos[2], 240.0)
     # まずは一言
    speaker.speak_sentence(_GREETING1[speaker.get_language()])

   
    # Greetings
    base.go(init_greeting_pos[0], init_greeting_pos[1], init_greeting_pos[2], 180.0)
    speaker.speak_sentence(_GREETING2[speaker.get_language()])
    speaker.speak_sentence(_GREETING3[speaker.get_language()])

    # 冷蔵庫
    base.go(refrigerator_pos[0], refrigerator_pos[1], refrigerator_pos[2], 180.0)
    speaker.speak_sentence(_REFRIGERATOR[speaker.get_language()])

    #本棚
    base.go(bookshelf_pos[0], bookshelf_pos[1], bookshelf_pos[2], 180.0)
    speaker.speak_sentence(_BOOKSHELF[speaker.get_language()])


    # 食器棚が見える場所に移動して一言
    base.go(shelf_front_pos[0], shelf_front_pos[1], shelf_front_pos[2], 180.0)
    speaker.speak_sentence(_SHELF1[speaker.get_language()])

    # バナナオーレが見える場所に移動して一言
    base.go(shelf_pos[0], shelf_pos[1], shelf_pos[2], _MOVE_TIMEOUT)
    speaker.speak_sentence(_SHELF2[speaker.get_language()])


    # バナナオーレを掴む
    try:
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        tts.say('初期化に失敗')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
        # ペットボトルの手前に手を持ってくる
        whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        # トルクを指定して把持する
        gripper.grasp(_GRASP_TORQUE)
        # シミュレータのgrasp hackのための待ち時間。実機では不要
        #rospy.sleep(2.0)
        # 手先相対で上にハンドを移動
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # 手先相対で後ろにハンドを移動
        whole_body.move_end_effector_pose(hand_back, _HAND_TF)
        # 初期姿勢に遷移
        whole_body.move_to_neutral()
    except:
        speaker.speak_sentence(_NG1[speaker.get_language()])
        rospy.logerr('fail to grasp')

    # バナナオーレを届ける
    base.go(greeting_pos[0], greeting_pos[1], greeting_pos[2], 180.0)

#    tts.say(u'次に床に落ちた紙を拾うよ。これってとっても難しいんだ。')
#    rospy.sleep(4)
#    # 紙が落ちている場所に移動する
#    base.go(paper_pos[0], paper_pos[1], paper_pos[2], 180.0)
#    # 紙を吸引する

#    # 見学者の側に移動する
#    base.go(greeting_pos[0], greeting_pos[1], greeting_pos[2], 180.0)


    # Start force sensor capture
    force_sensor_capture = ForceSensorCapture()

    # Set initial pose
    joint_controller = JointController()

    initial_position = JointState()
    initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'head_pan_joint',
                                  'head_tilt_joint', 'hand_motor_joint'])
    initial_position.position.extend([0.0, 0.0, 0.0, -1.57,
                                      0.0, 0.0, 0.0, 1.2])
    joint_controller.move_to_joint_positions(initial_position)

    # Get initial data of force sensor
    pre_force_list = force_sensor_capture.get_current_force()

    # Ask user to set object
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    rospy.sleep(2.0)

    # Inform user of next gripper action
    speaker.speak_sentence(_EXPLAIN2[speaker.get_language()])
    rospy.sleep(1.0)

    # Grasp the object
    joint_controller.grasp(-0.1)

    # Wait until force sensor data become stable
    rospy.sleep(1.0)
    post_force_list = force_sensor_capture.get_current_force()

    force_difference = compute_difference(pre_force_list, post_force_list)

    # Convert newton to gram
    weight = round(force_difference / 9.81 * 1000, 1)

    # Speak object weight in first decimal place
    speaker.speak_sentence(_ANSWER[speaker.get_language()].format(weight))

    # 最後に一言
    speaker.speak_sentence(_BY[speaker.get_language()])


