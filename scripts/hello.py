#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
#omni_base = robot.get('omni_base')
#whole_body = robot.get('whole_body')
#gripper = robot.get('gripper')
#tts = robot.get('default', robot.Items.TEXT_TO_SPEECH)
tts = robot.get('default_tts')
tts.language=tts.ENGLISH

if __name__=='__main__':

    # まずは一言
    rospy.sleep(5.0)
    #tts.say('こんにちはHSRだよ。発話のテストをするよ。')
    tts.say('Hello. This is super robot HSR')
    rospy.sleep(5.0)
