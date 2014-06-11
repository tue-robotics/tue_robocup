Challenge RoboZoo Simple

The robot does random things, like dancing and telling jokes.
Make sure it has plenty of space to the arms cant hit anything. 

amiddle and ahardware need to run

amigo2:
	$ ahardware

amigo1:
	$ amiddle

	#roslaunch qr_detector_zbar qr_data_publisher.launch
	#roslaunch rgbd_transport top_kinect_server.launch

Run the challenge on amigo3:
	$ sshamigo3
	$ rosrun challenge_robo_zoo robo_zoo_simple.py
