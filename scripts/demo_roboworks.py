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
#import paths
import tf
import re
import hsrb_interface

from hsrb_interface import Robot
from hsrb_interface import geometry
from hsrb_interface import ItemTypes
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray, String

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
# ROSは右手座標系
standby_pos = (-1.94, 0.0, 0) # 待機場所の座標
zero_pos = (0, 0.0, 0.0) # 原点
init_greeting_pos = (0.5, 0.0, 1.57) # 見学者に挨拶する場所
refrigerator_pos = (-0.4, -1.1, -1.57) # 冷蔵庫
bookshelf_pos = (4.21, -1.35, -1.57) # 本棚の場所
shelf_front_pos = (1.14, 1.65, 3.14) # 食器棚の見える場所
shelf_pos = (-0.15, 0.92, 3.14) # 食器棚の場所
paper_pos = (2.8, 1.4, 0) # 紙が落ちている場所
greeting_pos = (1.14, 1.65, 1.57) # 見学者に挨拶する場所


def go_and_say(pos=(0,0,0), contents=''):
    try:
        base.go(pos[0], pos[1], pos[2], _MOVE_TIMEOUT)
    except:
        rospy.logerr('Fail go')
    tts.say(contents)
    rospy.sleep(5)

_SENARIO = [
    (init_greeting_pos, u'みなさん改めてこんにちは。僕は家庭でみなさんの手伝いするために作られたロボットなんだ。飲み物を取ってきたり、床に落ちたゴミを拾ったりできるんだよ。まずは、ここロボット工房の案内をするよ。'),
    (refrigerator_pos, u'ここは冷蔵庫だよ。暑い夏には冷たい飲み物が欠かせないよね'),
    (bookshelf_pos, u'この本棚には学生さんたちの教科書がはいっているよ。学生は勉強が一番だからね。')]




if __name__=='__main__':
    # 初期姿勢に遷移
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')

    # 自己位置（ドア前）を地図上の位置に一致させる
    tts.say(u"自己位置を地図上の位置に一致させます。")
    rospy.sleep(2)

#    init_x = rospy.get_param("~init_x", 0.0)
#    init_y = rospy.get_param("~init_y", 0.0)
#    init_angle = rospy.get_param("~init_angle", 0.0)    
    init_x = standby_pos[0]
    init_y = standby_pos[1]
    init_angle = standby_pos[2]

    tts.say(u"自己位置のエックス座標は"+str(init_x)+u"ですね")
    rospy.sleep(3)
    tts.say(u"自己位置のワイ座標は"+str(init_y)+u"ですね")
    rospy.sleep(3)
    tts.say(u"自己位置の姿勢は"+str(init_angle)+u"ですね")
    rospy.sleep(3)

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
    rospy.sleep(15)



    # ドアの前でスタンバイ
    tts.say(u'イレイサー、準備完了。')
    rospy.sleep(2)

    # 腕を押したらタスクを開始する
    while True:
        wrench = robot.get('wrist_wrench',ItemTypes.FORCE_TORQUE).wrench
        if wrench[0][0] > 20.0:
            break
        pass
    tts.say(u'ロボット工房の案内プログラムを起動しました。ドアを開けてください。')
    rospy.sleep(4)

    # ドアが開いたら部屋に入る


    tts.say(u'ドアがあきました。部屋に入ります。')
    rospy.sleep(2)


    # 原点に移動
    base.go(zero_pos[0], zero_pos[1], zero_pos[2], 180.0)
 

    # まずは一言
    tts.say(u'こんにちは、僕はイレイサーだよ。これからロボット工房を案内するね。')
    rospy.sleep(3)


    #############
    #############
#    tts.say(u'ロボット工房の案内を中止します。')
#    rospy.sleep(3)
#    base.go(standby_pos[0], standby_pos[1], standby_pos[2], 180.0)
#    sys.exit()
    #############
    #############

    # 登録地点を順番に案内する
    for unit in _SENARIO:
        go_and_say(unit[0], unit[1])

    # 食器棚が見える場所に移動して一言
    base.go(shelf_front_pos[0], shelf_front_pos[1], shelf_front_pos[2], 180.0)
    tts.say(u'さて、これから僕のすごいところを見せちゃうよ。棚にある飲み物を取って、届けるよ。何がいいかなぁ、じゃあバナバオーレにしよう。僕はバナナオーレが好きなんだ。')
    rospy.sleep(6)

    # バナナオーレが見える場所に移動して一言
    base.go(shelf_pos[0], shelf_pos[1], shelf_pos[2], _MOVE_TIMEOUT)
    tts.say(u'バナナオーレを見つけました。')
    rospy.sleep(2)

    # バナナオーレを掴む
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
        tts.say('残念、失敗しました。バナナオーレを掴めませんでした。')
        rospy.sleep(3)
        rospy.logerr('fail to grasp')

    # バナナオーレを届ける
    base.go(greeting_pos[0], greeting_pos[1], greeting_pos[2], 180.0)


    tts.say(u'次に床に落ちた紙を拾うよ。これってとっても難しいんだ。')
    rospy.sleep(4)
    # 紙が落ちている場所に移動する
    base.go(paper_pos[0], paper_pos[1], paper_pos[2], 180.0)
    # 紙を吸引する

    # 見学者の側に移動する
    base.go(greeting_pos[0], greeting_pos[1], greeting_pos[2], 180.0)
    # 最後に一言
    tts.say(u'皆さん、今日はロボット工房まで来てくれてありがとう。またお会いしましょう。質問は岡田先生にしてね')
    rospy.sleep(5)
