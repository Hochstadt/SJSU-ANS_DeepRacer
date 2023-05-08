#!/usr/bin/env bash

CUR_PATH=`pwd`
DATA_PATH="${CUR_PATH}/data"
echo ${CUR_PATH}
if [ $# -ne 2 ]
then
  echo "Need to call this with IP address of deepracer as first argument and location of folder as second argument"
  echo "Ex: ./data_collector/retrieve_data.sh 192.168.1.26 /media/storage/good_collect"
else
  DEEPRACER_IP=$1
  MEDIA_LOCATION=$2
  #Check for existence of data directory
  if [ ! -d ${DATA_PATH} ]
  then
    mkdir ${DATA_PATH}
  fi
  cd ${DATA_PATH}
  #Attempt scp from deepracer
  echo "SCP-ing: deepracer@${DEEPRACER_IP}:${MEDIA_LOCATION}"
  #scp -r deepracer@${DEEPRACER_IP}:${MEDIA_LOCATION} .
  
  #Now go and run python script to convert pc2 to numpy arrays for map building
  #Extract folder name
  IFS='/'
  #Split wordsi nto array based on delimeter
  read -a strarr <<< ${MEDIA_LOCATION}
  IFS=' '
  words_num=${#strarr[*]}
  LOCAL_FOLDER=${strarr[${words_num}-1]}
  python3 ${CUR_PATH}/data_collector/data_collector/convert_to_data_list.py ${LOCAL_FOLDER}
  python3 ${CUR_PATH}/icp_map_building/icp_map_builder.py ${LOCAL_FOLDER}
  #Now copy the files you've created and to the navigation_module/navigator_host
  mkdir ${CUR_PATH}/algorithm_data
  cp ${DATA_PATH}/${LOCAL_FOLDER}/occupancy* ${CUR_PATH}/algorithm_data
  cp ${DATA_PATH}/${LOCAL_FOLDER}/map_file* ${CUR_PATH}/algorithm_data

fi

