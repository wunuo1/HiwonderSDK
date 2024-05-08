#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import threading
import sqlite3 as sql
from . import Board

#上位机编辑的动作调用库
custom_action_groups = ["pick_up", "back_small_step"]

runningAction = False
stop_action = False
stop_action_group = False

def stopAction():
    global stop_action
    
    stop_action = True

def stopActionGroup():
    global stop_action_group
    
    stop_action_group = True 

__end = False
__start = True
current_status = ''
def runActionGroup(actName, times=1, with_stand=False, lock_servos='', path="/home/pi/TonyPi/ActionGroups/", custom_path="/home/pi/TonyPi/CustomActionGroups"): 
    if actName in custom_action_groups:
        py_path = os.path.join(custom_path, actName + ".py")
        if os.path.exists(py_path):
            exec(open(py_path).read())
        else:
            print("未能找到动作组文件:{},是否将动作组保存在目录:{}".format(actName, custom_path))
    else:
        global __end
        global __start
        global current_status
        global stop_action_group
        
        temp = times
        while True:
            if temp != 0:
                times -= 1
            try:
                if (actName != 'go_forward' and actName != 'go_forward_fast' and actName != 'go_forward_slow' and actName != 'back' and actName != 'back_fast') or stop_action_group:
                    if __end:
                        __end = False
                        if current_status == 'go':
                            runAction('go_forward_end', lock_servos, path=path)
                        else:
                            runAction('back_end', lock_servos, path=path)
                        #print('end2')
                    if stop_action_group:
                        __end = False
                        __start = True
                        stop_action_group = False                        
                        #print('stop_action_group')
                        break
                    __start = True
                    if times < 0:
                        __end = False
                        __start = True
                        stop_action_group = False 
                        break
                    runAction(actName, lock_servos, path=path)
                else:
                    if times < 0:
                        #print('end1')
                        if with_stand:
                            if actName == 'go_forward' or actName == 'go_forward_fast' or actName == 'go_forward_slow':
                                runAction('go_forward_end', lock_servos, path=path)
                            else:
                                runAction('back_end', lock_servos, path=path)
                        break
                    if __start:
                        __start = False
                        __end = True
                        #print('start')
                        if actName == 'go_forward' or actName == 'go_forward_slow':                       
                            runAction('go_forward_start', lock_servos, path=path)
                            current_status = 'go'
                        elif actName == 'go_forward_fast':
                            runAction('go_forward_start_fast', lock_servos, path=path)
                            current_status = 'go'
                        elif actName == 'back':
                            runAction('back_start', lock_servos, path=path)
                            runAction('back', lock_servos, path=path)
                            current_status = 'back'                    
                        elif actName == 'back_fast':
                            runAction('back_start', lock_servos, path=path)
                            runAction('back_fast', lock_servos, path=path)
                            current_status = 'back'
                    else:
                        runAction(actName, lock_servos, path=path)
            except BaseException as e:
                print(e)

def runAction(actNum, lock_servos='', path="/home/pi/TonyPi/ActionGroups/"):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :return:
    '''
    global runningAction
    global stop_action
    
    if actNum is None:
        return

    actNum = path + actNum + ".d6a"

    if os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            while True:
                act = cu.fetchone()
                if stop_action is True:
                    stop_action = False
                    print('stop')                    
                    break
                if act is not None:
                    for i in range(0, len(act) - 2, 1):
                        if str(i + 1) in lock_servos:
                            Board.setBusServoPulse(i + 1, lock_servos[str(i + 1)], act[1])
                        else:
                            Board.setBusServoPulse(i + 1, act[2 + i], act[1])
                    time.sleep(float(act[1])/1000.0)
                else:   # 运行完才退出
                    break
            runningAction = False
            
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件:(),是否将动作组保存在目录:{}".format(actNum, path))
