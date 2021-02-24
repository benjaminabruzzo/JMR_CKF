# Gatorboard mono:
	rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 image:=/ardrone/bottom/image_raw camera:=/ardrone/bottom

# Gatorboard stereo:
	on ugv:
		. ~/ros/src/metahast/scripts/makedirs.sh 20200122 1 20
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_base.launch ugv_n:=ugv1

		rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/ugv1/pgrstereo/right/image_raw left:=/ugv1/pgrstereo/left/image_raw right_camera:=/ugv1/pgrstereo/right left_camera:=/ugv1/pgrstereo/left --approximate=0.005
		rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/ugv2/pgrstereo/right/image_raw left:=/ugv2/pgrstereo/left/image_raw right_camera:=/ugv2/pgrstereo/right left_camera:=/ugv2/pgrstereo/left --approximate=0.005

		save output to ~/ros/src/metahast/hast/cam_info/calibration/ugvn_date.ini

		in ugvCalPGR change:
			cal.filename = 'ugv1_20200122';
			HOSTID = 'ugv1';
			Left.SERIAL  = '17496991';
			Right.SERIAL = '17496985';

		in matlab:
			matlab -nojvm -nodesktop -r "cd('/home/benjamin/ros/src/metahast/hast/cam_info');ugvCalPGR('linux')"

## then check the quality of calibration and determine the H_cam2ugv
	On vicon pc:
		. ~/ros/src/metahast/scripts/makedirs.sh 20200129 1 20
		roslaunch hast trial_params.launch date:=20200128 trial:=005
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch gator_w_ugvs.launch ugv_n:=ugv1
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch gator_w_ugvs.launch ugv_n:=ugv2

	on ugv:
		. ~/ros/src/metahast/scripts/makedirs.sh 20200122 1 20
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_base.launch ugv_n:=ugv1
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_base.launch ugv_n:=ugv2

	use this on the vicon pc to log the check point data with the ugv at different locations
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch test_calibration.launch ugv_n:=ugv1
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch test_calibration.launch ugv_n:=ugv2

	update date and trial numbers in finalize_calibration.m
	the output is the H_cam2ugv transform (but probably needs to have some +- signs adjusted, still not perfect)

## then transfer all files to ~/.ros
	cp  ~/ros/src/metahast/hast/cam_info/calibration/16306423.yaml ~/ros/src/metahast/hast/cam_info/
	cp  ~/ros/src/metahast/hast/cam_info/calibration/16369047.yaml ~/ros/src/metahast/hast/cam_info/
	# git upload/download
	cp  ~/ros/src/metahast/hast/cam_info/*.yaml /home/benjamin/.ros/camera_info/


#### Less good methods:

Gather Data
	On vicon pc:
		make necessary directories
			mkdir -p ~/ros/data/calibrations/20200117/ugv1/bad && mkdir -p ~/ros/data/calibrations/20200117/ugv1/raw
			mkdir -p ~/ros/data/calibrations/20200117/ugv1/rect && mkdir -p ~/ros/data/calibrations/20200117/ugv1/test_rect
		launch vicon
			cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch gator_w_ugvs.launch

	On ugv:
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_base.launch ugv_n:=ugv1
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_teleop.launch

Calibration between two cameras
# https://github.com/opencv/opencv/issues/11131
# There is a possible issue with openCV stereo recitfy: https://answers.opencv.org/question/93090/stereo-rectify-doesnt-rectify-even-with-correct-m-and-d/

	Using Matlab: (requires computer vision toolbox)
		cd ~/ros/src/metahast/hast/cam_info/calibration/src/
		matlabCalibration.m

		update config settings in
			atom ~/ros/src/metahast/hast/cam_info/calibration/yaml/calibrate_ugv_stereo.yaml
		then process data
			cd ~/ros/src/metahast/hast/cam_info/calibration/src && python calibrate_ugv_stereo.py
			cp ~/ros/data/calibrations/20200115/ugv1/*.yaml ~/ros/src/metahast/hast/cam_info/
			cp ~/ros/data/calibrations/20200115/ugv1/ugv1_pgr_params.yaml ~/ros/src/metahast/hast/cam_info/ugvn/

	on robot
		cp  ~/ros/src/metahast/hast/cam_info/*.yaml /home/benjamin/.ros/camera_info/



Test calibration
	On vicon pc:
		. ~/ros/src/metahast/scripts/makedirs.sh 20200122 1 20
		roslaunch hast trial_params.launch date:=20200122 trial:=001
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch gator_w_ugvs.launch
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch test_calibration.launch

	on ugv:
		. ~/ros/src/metahast/scripts/makedirs.sh 20200122 1 20
		cd ~/ros/src/metahast/hast/cam_info/calibration/launch && roslaunch cal_base.launch ugv_n:=ugv1


Sandboxing:
	cd ~/ros/src/metahast/hast/cam_info/calibration/src && python sandbox.py

	cd ~/ros/src/metahast/hast/cam_info/calibration/src && python sandbox.py

	cp ~/ros/src/metahast/hast/cam_info/17496991_python.yaml ~/.ros/camera_info/17496991.yaml
	cp ~/ros/src/metahast/hast/cam_info/17496985_python.yaml ~/.ros/camera_info/17496985.yaml
