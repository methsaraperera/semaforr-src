# Author: Anoop Aroor
# This is a python launch file to control and launch ros nodes for Semaforr project
# Each ros node must has a launch file containing parameters that remain constant though different experiments
# Dynamic parameters that changes with different experiments are added here

import rospy
import time
import subprocess

def experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params):
    project_home = "/home/rajkochhar/catkin_ws1/src"
    menge_path = project_home+"/examples/core"
    semaforr_path = project_home+"/semaforr"

    map_folder = menge_path+"/"+map_name
    map_xml = menge_path+"/"+map_name+".xml"

    #menge files for semaforr
    map_config = map_folder+"/"+map_name+"S.xml"
    map_dimensions = map_folder+"/dimensions.conf"
    target_set = map_folder+"/" + target_file_name


    print target_set
    print map_config
    print map_xml
    print map_dimensions
    print log_name
    #print why_log_name
    #print whyplan_log_name

    #start roscore
    roscore = subprocess.Popen(['roscore'])
    time.sleep(5)

    # start menge simulator
    menge_sim_process = subprocess.Popen(['rosrun','menge_sim','menge_sim','-p',map_xml])
    print "waiting,,"
    time.sleep(10)

    # start crowd model
    crowd_process = subprocess.Popen(['rosrun','crowd_learner','learn.py',density, flow, risk, cusum, discount, explore])

    # start logging
    log_file = open(log_name,"w")
    log_process = subprocess.Popen(['rostopic','echo','/decision_log'],stdout=log_file)

    #why_log_file = open(why_log_name,"w")
    #why_log_process = subprocess.Popen(['rostopic','echo','/plan_explanations_log'],stdout=why_log_file)

    #whyplan_log_file = open(whyplan_log_name,"w")
    #whyplan_log_process = subprocess.Popen(['rostopic','echo','/plan_explanations'],stdout=whyplan_log_file)

    # start semaforr
    semaforr_process = subprocess.Popen(['rosrun','semaforr','semaforr', semaforr_path, target_set, map_config, map_dimensions, advisors, params])
    print "waiting,,"

    # start why
    #why_process = subprocess.Popen(['rosrun','why','why'])
    #print "waiting,,"

    # start why_plan
    #why_plan_process = subprocess.Popen(['rosrun','why_plan','why_plan'])
    #print "waiting,,"
   
    rviz_process = subprocess.Popen(['rosrun','rviz','rviz'])

    # Wait till semaforr completes the process
    while semaforr_process.poll() is None:
        print "Semaforr process still running ..."
        if rviz_process.poll() is not None:
            rviz_process = subprocess.Popen(['rosrun','rviz','rviz'])
        time.sleep(1)

    print "Semaforr process has ended ..."
    print "Terminating the simulator"

    menge_sim_process.terminate()
    while menge_sim_process.poll() is None:
        print "Menge process still running ..."
        time.sleep(1)
    
    rviz_process.terminate()
    print "Menge terminated!"
    print "Terminating crowd model"
    crowd_process.terminate()
    #why_plan_process.terminate()
    log_process.terminate()
    log_file.close()
    #why_log_process.terminate()
    #why_log_file.close()
    #whyplan_log_process.terminate()
    #whyplan_log_file.close()
    time.sleep(1)
    #why_process.terminate()
    #print "Why terminated!"

    roscore.terminate()
    time.sleep(10)
    print "roscore terminated!"

map_name = "gradcenter-4"
density = "on"
flow = "on"
risk = "on"
cusum = "off"
discount = "off"
explore = "off"

for i in range(0,1):
    target_file_name = "target.conf"
    log_name = map_name + "_" + "astar" + "_" + str(i) + ".txt"
    advisors = "/config/advisors0.conf"
    params = "/config/params0.conf"
    #why_log_name = map_name + "_" + str(i) + "_why_log.txt"
    #whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
    experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params)

# for i in range(0,7):
#     target_file_name = "target.conf"
#     log_name = map_name + "_" + "astar_crowd_adv" + "_" + str(i) + ".txt"
#     advisors = "/config/advisors1.conf"
#     params = "/config/params1.conf"
#     #why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     #whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params)

# for i in range(0,7):
#     target_file_name = "target.conf"
#     log_name = map_name + "_" + "astar_prox_adv" + "_" + str(i) + ".txt"
#     advisors = "/config/advisors2.conf"
#     params = "/config/params2.conf"
#     #why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     #whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params)

# for i in range(0,7):
#     target_file_name = "target.conf"
#     log_name = map_name + "_" + "astar_crowd_prox_adv" + "_" + str(i) + ".txt"
#     advisors = "/config/advisors3.conf"
#     params = "/config/params3.conf"
#     #why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     #whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params)

# for i in range(0,7):
#     target_file_name = "target.conf"
#     log_name = map_name + "_" + "crowd_plans_crowd_prox_adv" + "_" + str(i) + ".txt"
#     advisors = "/config/advisors4.conf"
#     params = "/config/params4.conf"
#     #why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     #whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params)