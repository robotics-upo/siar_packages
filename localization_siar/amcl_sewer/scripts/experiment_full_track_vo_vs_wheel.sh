# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

CONTADOR=$1

if [ $# -ne 2 ]; then
  echo "Usage: $0 <first_exe_number> <last_exe_number>"
  exit 1
fi

# Initial Parameters_21_sept. 
initial_x=100.52
initial_y=-197.05
initial_a=-0.562
bag_file=/home/chur/Dataset/2017-09-21_full_track_surface/siar_2017-09-21-12-17-39.bag
ground_file=/home/chur/Dataset/2017-09-21_full_track_surface/input_vector_2017-09-21_ground_truth.txt
start=560

# Now with wheel odometry
# 
#  until [ $CONTADOR -gt $2 ]; do  
#    roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
#    ground_truth_out:=/home/chur/stats/stats21_sept_wheel_mod_0.1_mod_a_0.05_noise_0.05/stats_$CONTADOR.txt \
#   odom_a_mod:=0.05 odom_a_noise:=0.05 odom_x_mod:=0.1 odom_y_mod:=0.1 \
#   initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
#   let pid1=$!
#   rosbag play $bag_file -s $start --clock -r 2.2 -u 3260
#   rosnode kill -a
#   wait ${pid1}
#   let CONTADOR+=1
# done

# Visual Odometry
# CONTADOR=$1
# until [ $CONTADOR -gt $2 ]; do
#   # Roslaunch with multiple parameters
#   params=mod_0.5_a_mod_0.25_n_0.075
#   
#   roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
#   ground_truth_out:=/home/chur/stats/stats_detector_21_sept_vo_2_${params}_$CONTADOR.txt stats_file:=/home/chur/stats/stats21_sept_vo_2_${params}_$CONTADOR.txt \
#   odom_a_mod:=0.25 odom_a_noise:=0.075 odom_x_mod:=0.5 odom_y_mod:=0.5 \
#   camera:=/back initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=true &
#   
#   #end of roslaunch
#   
#   let pid1=$!
#   rosbag play $bag_file -s $start --clock -r 0.9
#   rosnode kill -a
#   wait ${pid1}
#   let CONTADOR+=1
# done

# NOTE: For adjusting noise settings
#   odom_a_mod:=0.25 odom_a_noise:=0.075 odom_x_mod:=0.5 odom_y_mod:=0.5 \

# With yaw estimation
CONTADOR=$1
until [ $CONTADOR -gt $2 ]; do
  # Roslaunch with multiple parameters
  roslaunch amcl_sewer amcl_bag.launch play_bag:=false ground_truth:=$ground_file \
  ground_truth_out:=/home/chur/stats/detector_21_sept_wheel_estimate_yaw_debugged_mod_0.1_mod_a_0.05_noise_0.05/stats_$CONTADOR.txt  \
  yaw_estimator:=true \
  odom_a_mod:=0.05 odom_a_noise:=0.05 odom_x_mod:=0.1 odom_y_mod:=0.1 \
  camera:=/front initial_x:=$initial_x initial_y:=$initial_y initial_a:=$initial_a rgbd_odom:=false &
  
  #end of roslaunch
  
  let pid1=$!
  rosbag play $bag_file -s $start --clock -r 2.2 -u 3260
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1
done