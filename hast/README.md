# always start by preparing the directory tree
. ~/ros/src/metahast/scripts/makedirs.sh 20200514 1 10

# Launching an experiment in VICON
## 1) on the vicon PC launch the vicon bridge, vicon logger, rviz conifgs.  This is essentially the same as spawn_robots
	roslaunch hast init.launch date:=20200318 trial:=003

## 2) on each ugv, launch the vehicle nodes:
### always start by preparing the directory tree
	. ~/ros/src/metahast/scripts/makedirs.sh 20200313 1 20
### launch the ugv nodes
	roslaunch hast kobuki_base_n.launch ugv_n:=ugv1
	roslaunch hast kobuki_base_n.launch ugv_n:=ugv2 primary:=false

## 3) on the vicon PC, launch the experiment
	roslaunch hast hast_ardrone.launch
	roslaunch hast hast_april_bottomcam.launch
	roslaunch hast vicon_trial.launch

## 4) pull data to vicon pc
	. ~/ros/src/metahast/scripts/pull_from.sh mk3a 20200313
	. ~/ros/src/metahast/scripts/pull_from.sh mk3 20200313
	. ~/ros/src/metahast/scripts/pull_to_ares.sh 20200318 -x

### SSH into ground vehicle
	. ~/ros/src/metahast/scripts/push_to_ares.sh 20200313 -x

# Launch multiple experiments:
cd ~/ros/src/metahast/scripts/ && . specific_trials.sh

	runs gazebo_iterant.sh
		roscore
		hast_gazebo init.launch
			empty_world.launch
		hast_gazebo init_uav_rviz.Launch
			$(find hast)/launch/vicon_logger.launch
			$(find hast)/launch/rvis.launch
			$(find hast_gazebo)/launch/gazebo_uav.launch
				$(find hast)/launch/hast_uavAutopilot.launch
				$(find hast)/launch/hast_april_bottomcam.launch
		hast_gazebo ugv1_base.launch
			$(find robot_descriptions)/UGV_n/launch/spawn_ugv.launch
			$(find hast)/launch/hast_ugv.launch
				$(find hast)/launch/hast_stereo_obs.launch
				$(find hast)/launch/hast_ugvCKF.launch
				$(find hast)/launch/ugvMuxxer.launch
		hast_gazebo ugv2_base.launch
			$(find robot_descriptions)/UGV_n/launch/spawn_ugv.launch
			$(find hast)/launch/hast_ugv.launch
				$(find hast)/launch/hast_stereo_obs.launch
				$(find hast)/launch/hast_ugvCKF.launch
				$(find hast)/launch/ugvMuxxer.launch
		hast_gazebo gazebo_trial.launch
			$(find hast)/launch/hast_jointSLAM.launch
			uav_topic_switch.py
			$(find hast)/launch/trial.launch
		hast wait_for_trial_finish.launch
		hast countdown.launch

# Launching one trial
	DATE=20200728; EXP_CODE=H; TRIAL=211
	mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/config";
	cp /home/benjamin/ros/src/metahast/robot_descriptions/tags/launch/tags_rand.launch /home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/config/tags_rand.launch
	mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/prealloc"
	mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/figs/eps"
	mkdir -p "/home/benjamin/ros/data/$DATE/$EXP_CODE/$TRIAL/figs/png"
	roslaunch hast_gazebo init.launch date:=$DATE exp_code:=$EXP_CODE trial:=$TRIAL

	roslaunch hast_gazebo init_uav_rviz.launch
	roslaunch hast_gazebo ugv1_base.launch exp_code:=H
	roslaunch hast_gazebo ugv2_base.launch exp_code:=H

	roslaunch hast_gazebo gazebo_trial.launch oneUGV:=false ugv2_lookatgoal:=false ugv2_watchugv1:=true  ugv1_picket:=false ugv1_w_hover:=true


# Launching trials on remotes:
## remote call
	cd ~/ros/src/metahast/scripts && . remote_trials.sh atlas


# Launching an experiment in GAZEBO
## Batch Launching an experiment in GAZEBO
	cd ~/ros/src/metahast/scripts && 	. iterate_trials.sh 20200523 2 20
	cd ~/ros/src/metahast/scripts && 	. iterate_allrand.sh 20200513 1 3
	cd ~/ros/src/metahast/scripts && 	. iterate_rand.sh


	<!-- Good maps to use: 10, 16, 19, 20, 22,
	Maybe: 21, 23, 25 -->

### Launch a single experiment with code A-F
#### First copy landmark map file
	src_map=/home/benjamin/ros/data/20200610/tags_010.launch; cp $src_map /home/benjamin/ros/src/metahast/robot_descriptions/tags/launch/tags_rand.launch
	cd ~/ros/src/metahast/scripts && 	. single_trial.sh 20200610 10 A



## 1) launch the gazebo environment and robot config
	roslaunch hast_gazebo init.launch date:=20200520 exp_code:=A trial:=001

## 2) launch the vehicle nodes:
	roslaunch hast_gazebo ugv1_base.launch exp_code:=A
	roslaunch hast_gazebo ugv2_base.launch absolute_init:=true

## 3) launch the experiment
	roslaunch hast_gazebo trial.launch oneUGV:=false ugv2_picket:=true
	( roslaunch hast wait_for_trial_finish.launch )



mkdir -p "/home/benjamin/ros/data/20200626/F/007/config";
mkdir -p "/home/benjamin/ros/data/20200626/F/007/prealloc"
mkdir -p "/home/benjamin/ros/data/20200626/F/007/figs/eps"
mkdir -p "/home/benjamin/ros/data/20200626/F/007/figs/png"


# (OLD) Launching an experiment
## 1) start the uav process:
	roslaunch hast ardrone.launch

## 2) launch the UGV nodes:
	roslaunch hast kobuki_base_n.launch

## 3) launch the recorder and KF files with dates
	roslaunch hast hast_n.launch date:=20180418 trial:=001
### 3a) flags:
	saveimages:=true 	:: saves images
	showcircles:=true 	:: shows left and right images with "found" circles
	<arg name="sim" 		default="false" />
	<arg name="april_debug" default="false" />
	<arg name="trial" 		default="000" />
	<arg name="date" 		default="20160930" />
	<arg name="user" 		default="$(env USER)" />
	<arg name="saveimages" 	default="false" />
	<arg name="showcircles" default="false" />
	<arg name="saveraw" 	default="false" />
	<arg name="trigger_saveraw"	default="false" />  <!-- rosrun hast triggerSaveRaw -->
	<arg name="tb_base" 	default="kobuki" />
	<arg name="down_image"  default="image_rect" />
	<arg name="tf_x_offset" default="0.025" />

## 4) launch the experiment
	roslaunch hast experiment.launch action:=true step:=true
### 4a) flags:
	abcde:=true 	:: UAV flys through waypoints A-E
	<arg name="arc" 		default="false" />
	<arg name="pull" 		default="false" />
	<arg name="picket" 		default="false" />
	<arg name="abcde" 		default="false" />
	<arg name="step" 		default="false" />
	<arg name="ugvfore" 	default="false" />
	<arg name="ugvauto" 	default="false" />
	<arg name="iflift" 		default="false" />
	<arg name="aprillog" 	default="false" />
	<arg name="ugvcomplex" 	default="false" />
	<arg name="action" 		default="false" />

## 5) TRENDNET_CREATE::turtlebot_create
