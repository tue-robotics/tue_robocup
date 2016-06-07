# Responsible: Rein

amigo1/sergio1:

    - astart/sstart # (if not already running)

amigo2/sergio1:

    - ahardware/shardware: base, spindle, arms, head

amigo1/sergio1:

    - amiddle/smiddle 

amigo1:

    - The robot has to be positioned in the correct location before starting the executive! Place him exactly with the back facing the crowd.
    - make sure you have mounted the USB @ /home/amigo/usb
    - rosrun challenge_person_recognition person_recognition.py [robot_name] # On AMIGO1 !!
