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


until [ $CONTADOR -gt $2 ]; do
  
  roslaunch amcl_sewer amcl_experiment_11_fake_detector.launch stats_file:=/home/chur/stats/stats11_cnn_$CONTADOR.txt &
  let  pid1 = $!
  rosbag play "/home/chur/SIAR/Sewers_jan_17/siar_2017-01-17-11-17-28.bag" -s 390 --clock -r 0.9
  rosnode kill -a
  wait ${pid1}
  let CONTADOR+=1

done


