# Author: Anoop Aroor
# This is a python launch file to control and launch ros nodes for Semaforr project
# Each ros node must has a launch file containing parameters that remain constant though different experiments
# Dynamic parameters that changes with different experiments are added here

import rospy
import time
import subprocess

def experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations):
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
    print why_log_name
    print whyplan_log_name
    print situation_log_name

    #start roscore
    roscore = subprocess.Popen(['roscore'])
    time.sleep(5)

    # start menge simulator
    menge_sim_process = subprocess.Popen(['rosrun','menge_sim','menge_sim','-p',map_xml])
    print "waiting,,"
    time.sleep(30)

    # start crowd model
    #crowd_process = subprocess.Popen(['rosrun','crowd_learner','learn.py',density, flow, risk, cusum, discount, explore])

    # start situations
    # situation_process = subprocess.Popen(['rosrun','situation_learner','learn.py'])

    # start logging
    # situation_log_file = open(situation_log_name,"w")
    # situation_log_process = subprocess.Popen(['rostopic','echo','/situations'],stdout=situation_log_file)

    # start logging
    log_file = open(log_name,"w")
    log_process = subprocess.Popen(['rostopic','echo','/decision_log'],stdout=log_file)

    # why_explanations_file = open(why_explanations_name,"w")
    # why_explanations_process = subprocess.Popen(['rostopic','echo','/explanations'],stdout=why_explanations_file)

    # whyplan_explanations_file = open(whyplan_explanations_name,"w")
    # whyplan_explanations_process = subprocess.Popen(['rostopic','echo','/plan_explanations'],stdout=whyplan_explanations_file)

    # why_log_file = open(why_log_name,"w")
    # why_log_process = subprocess.Popen(['rostopic','echo','/explanations_log'],stdout=why_log_file)

    # whyplan_log_file = open(whyplan_log_name,"w")
    # whyplan_log_process = subprocess.Popen(['rostopic','echo','/plan_explanations_log'],stdout=whyplan_log_file)

    # start semaforr
    semaforr_process = subprocess.Popen(['rosrun','semaforr','semaforr', semaforr_path, target_set, map_config, map_dimensions, advisors, params, situations])
    print "waiting,,"
    time.sleep(2)
    
    # start people_trajectories
    # people_trajectories_process = subprocess.Popen(['rosrun','people_trajectories','people_trajectories'])
    # print "waiting,,"

    # start why
    # why_process = subprocess.Popen(['rosrun','why','why'])
    # print "waiting,,"

    # start why_plan
    # why_plan_process = subprocess.Popen(['rosrun','why_plan','why_plan'])
    # print "waiting,,"
   
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
    print "Menge terminated!"

    # people_trajectories_process.terminate()

    rviz_process.terminate()
    
    # print "Terminating crowd model"
    #crowd_process.terminate()
    # situation_process.terminate()
    # why_process.terminate()
    # why_plan_process.terminate()
    # print "Why terminated!"
    log_process.terminate()
    log_file.close()
    # why_explanations_process.terminate()
    # why_explanations_file.close()
    # whyplan_explanations_process.terminate()
    # whyplan_explanations_file.close()
    # why_log_process.terminate()
    # why_log_file.close()
    # whyplan_log_process.terminate()
    # whyplan_log_file.close()
    # situation_log_process.terminate()
    # situation_log_file.close()
    time.sleep(1)

    roscore.terminate()
    time.sleep(10)
    print "roscore terminated!"

map_name = "map-a"
density = "on"
flow = "on"
risk = "on"
cusum = "off"
discount = "off"
explore = "off"

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_sit_plan_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params8.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_sit_noplan_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params9.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_fullplan_nosit_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params2.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_fullplan_sit_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params3.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 4
# for i in range(3,num_runs):
#     target_file_name = "target40.conf"
#     log_name = map_name + "_skeleton_plan_" + str(i) + ".txt"
#     advisors = "/config/advisors1.conf"
#     params = "/config/params4.conf"
#     situations = "/config/situations3.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

map_name = "gradcenter-5"
num_runs = 3
for i in range(2,num_runs):
    target_file_name = "targettraintest.conf"
    log_name = map_name + "_highwayexplore_plan_" + str(i) + ".txt"
    advisors = "/config/advisors1.conf"
    params = "/config/params4.conf"
    situations = "/config/situations3.conf"
    why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
    whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
    why_log_name = map_name + "_" + str(i) + "_why_log.txt"
    whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
    situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
    experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_spatial_sit_noplan_" + str(i) + ".txt"
#     advisors = "/config/advisors1.conf"
#     params = "/config/params5.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(0,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_spatial_nosit_plan_" + str(i) + ".txt"
#     advisors = "/config/advisors1.conf"
#     params = "/config/params6.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(4,num_runs):
# for i in [9]:
#     target_file_name = "target100.conf"
#     log_name = map_name + "_spatial_nosit_noplan_" + str(i) + ".txt"
#     advisors = "/config/advisors1.conf"
#     params = "/config/params7.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# # num_runs = 10
# # for i in range(0,num_runs):
# for i in [5, 8]:
#     target_file_name = "target100.conf"
#     log_name = map_name + "_nosit_plan_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params0.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)

# num_runs = 10
# for i in range(1,num_runs):
#     target_file_name = "target100.conf"
#     log_name = map_name + "_nosit_noplan_" + str(i) + ".txt"
#     advisors = "/config/advisors0.conf"
#     params = "/config/params1.conf"
#     situations = "/config/situations2.conf"
#     why_explanations_name = map_name + "_" + str(i) + "_why_explanations.txt"
#     whyplan_explanations_name = map_name + "_" + str(i) + "_why_plan_explanations.txt"
#     why_log_name = map_name + "_" + str(i) + "_why_log.txt"
#     whyplan_log_name = map_name + "_" + str(i) + "_why_plan_log.txt"
#     situation_log_name = map_name + "_" + str(i) + "_situation_log.txt"
#     experiment(map_name, log_name, density, flow, risk, cusum, discount, explore, advisors, params, situations)