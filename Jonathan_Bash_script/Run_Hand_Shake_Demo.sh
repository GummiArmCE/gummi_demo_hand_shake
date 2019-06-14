#!/bin/bash

#Bash Program to run Gummi Arm gummi_demo_hand_shake

Survey_manager() {
  sleep 2
  exec 3< <(rostopic echo /rosout_agg)
  while read line; do
     case "$line" in
     *"gummi_d: Found 13 motors"*)
        echo "Manager setup completed"
        break
        ;;
     *)

        ;;
     esac
  done <&3

  # Close the file descriptor
  exec 3<&-
}

Survey_gummi() {
  sleep 2
  exec 3< <(rostopic echo /rosout_agg)
  while read line; do
     case "$line" in
     *"GummiArm is live!"*)
        echo "Gummi setup completed"
        break
        ;;
     *)

        ;;
     esac
  done <&3

  # Close the file descriptor
  exec 3<&-
}
echo "Running gummi_demo_handshake"
cd /home/user/demos/handshake_demo/
source devel/setup.bash

echo "Running command  \"roslaunch gummi_base_template manager.launch\""
roslaunch gummi_base_template manager.launch > /dev/null & Survey_manager
sleep 2
echo "Running command  \"roslaunch gummi_base_template controllers.launch\""
roslaunch gummi_base_template controllers.launch  > /dev/null
echo "Controllers setup completed"
sleep 2
echo "Running command  \"roslaunch gummi_interface gummi.launch\""
roslaunch gummi_interface gummi.launch > /dev/null & Survey_gummi
sleep 4
echo "Opening Display"
echo "Running command  \"roslaunch gummi_demo_handshake hand_shake_all.launch\""
#roslaunch gummi_demo_hand_shake display_LIVE.launch &
roslaunch gummi_demo_hand_shake hand_shake_all.launch
echo "Program Terminating"
sleep 10

#command > /dev/null 2>&1
