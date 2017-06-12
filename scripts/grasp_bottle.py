#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持トルク[Nm]
_GRASP_TORQUE=-0.01
# ボトルのtf名
_BOTTLE_TF='ar_marker/1'
# グリッパのtf名
_HAND_TF='hand_palm_link'

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# bottleのマーカーの手前0.02[m],z軸回に-1.57回転させた姿勢
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# handを0.1[m]上に移動させる姿勢
hand_up = geometry.pose(x=0.1)

# handを0.5[m]手前に移動させる姿勢
hand_back = geometry.pose(z=-0.5)

# ソファの場所
sofa_pos = (0, 0.0, 0)

if __name__=='__main__':

    # まずは一言
    rospy.sleep(10.0)
    tts.say('こんにちはHSRだよ。bananaを掴もうと思います。')
    rospy.sleep(5.0)

    try:
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        tts.say('初期化に失敗')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # ペットボトルが見える場所に移動
        omni_base.go(sofa_pos[0], sofa_pos[1], sofa_pos[2], _MOVE_TIMEOUT)
    except:
        tts.say('移動に失敗')
        rospy.logerr('fail to move')
        sys.exit()

    try:
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
        # ペットボトルの手前に手を持ってくる
        whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        # トルクを指定して把持する
        gripper.grasp(_GRASP_TORQUE)
        # シミュレータのgrasp hackのための待ち時間。実機では不要
        rospy.sleep(2.0)
        # 手先相対で上にハンドを移動
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # 手先相対で後ろにハンドを移動
        whole_body.move_end_effector_pose(hand_back, _HAND_TF)
        # 初期姿勢に遷移
        whole_body.move_to_neutral()
    except:
        tts.say('把持失敗')
        rospy.logerr('fail to grasp')
        sys.exit()
