#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import Robot
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

# 棚の場所
shelf_pos = (0, 0.0, 0)

def go_and_say(pos=(0,0,0), contents=''):
    try:
        base.go(pos[0], pos[1], pos[2], _MOVE_TIMEOUT)
    except:
        rospy.logerr('Fail go')
    tts.say(contents)
    rospy.sleep(5)

_SENARIO = [
    ((2.9, 0.36, -1.57, 180,0), u'みなさん改めてこんにちは。僕は家庭でみなさんの手伝いするために作られたロボットなんだ。飲み物を取ってきたり、床に落ちたゴミを拾ったりできるんだよ。まずは、ここロボット工房の案内をするよ。'),
    ((5.4, 0.07, -1.57, 180,0), u'ここは冷蔵庫だよ。暑い夏には冷たい飲み物が欠かせないよね'),
    ((5.7, 1.6, 3.14, 180,0), u'この本棚には学生さんたちの教科書がはいっているよ。学生は勉強が一番だからね。'),
    ((1.5, 3.3, 1.57, 180,0), u'さて、これから僕のすごいところを見せちゃうよ。棚にある飲み物を取って、届けるよ。何がいいかなぁ、じゃあバナバオーレにしよう。僕はバナナオーレが好きなんだ。'),


if __name__=='__main__':
    # 初期姿勢に遷移
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')

    # まずは一言
    tts.say(u'こんにちは、僕はイレイサーだよ。これからロボット工房を案内するね。')

    for unit in _SENARIO:
        go_and_say(unit[0], unit[1])


    # バナナオーレが見える場所に移動
    omni_base.go(shelf_pos[0], shelf_pos[1], shelf_pos[2], _MOVE_TIMEOUT)
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
        tts.say('残念、失敗しました。バナナオーレが見つかりません。')
        rospy.logerr('fail to grasp')

    # バナナオーレを届ける
    base.go(0.0, 0.0, 3.14, 180.0)


    tts.say(u'次に床に落ちた紙を拾うよ。これってとっても難しいんだ。')
    # 紙が落ちている場所に移動する
    base.go(0.0, 0.0, 3.14, 180.0)
    # 紙がを吸引する

    # 見学者の側に移動する
    base.go(0.0, 0.0, 3.14, 180.0)
    # 最後に一言
    tts.say(u'皆さん、今日はロボット工房まで来てくれてありがとう。またお会いしましょう。質問は岡田先生にしてね')
