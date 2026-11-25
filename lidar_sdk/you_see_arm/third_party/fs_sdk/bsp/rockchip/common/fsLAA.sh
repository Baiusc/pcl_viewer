#!/bin/bash
ROOT_DIR="/fortsense/fslidar"
UPDATE_DIR="/data/fslidar"

if [ -f "${UPDATE_DIR}/.DEBUG" ] ; then
    echo "Found '${UPDATE_DIR}/.DEBUG'. Current Debug Mode, Can't start App" | tee -a /var/log/messages
    while true ; do
        if [ -f "${UPDATE_DIR}/.DEBUG" ] ; then
            sleep 1 ;
        else
            echo "Lost  '${UPDATE_DIR}/.DEBUG'. Debug Mode Closed, Start App" | tee -a /var/log/messages
            break
        fi
    done
fi

# 如果存在更新目录中的启动脚本，且当前脚本不是该文件，则切换到更新脚本执行
SELF_PATH="$(readlink -f "$0" 2>/dev/null || echo "$0")"
UPDATE_SCRIPT="${UPDATE_DIR}/bin/fsLAA.sh"
if [ -f "${UPDATE_SCRIPT}" ] ; then
     if [ "${SELF_PATH}" != "${UPDATE_SCRIPT}" ] ; then
          echo "use ${UPDATE_DIR}/bin/fsLAA.sh" | tee -a /var/log/messages
          exec "${UPDATE_SCRIPT}"
          exit $?
     fi
fi

export PATH="${UPDATE_DIR}/bin:${ROOT_DIR}/bin:/bin:/sbin:/usr/bin:/usr/sbin"
export LD_LIBRARY_PATH="${UPDATE_DIR}/lib:${ROOT_DIR}/lib:/lib:/usr/lib"

# [ -f ${ROOT_DIR}/bin/app                           ] || ( echo "${ROOT_DIR}/bin/app not exist!"                           | tee -a /var/log/messages && exit 1 ) || exit 1
# [ -f /lib/modules/5.15.147/videobuf2-dma-contig.ko ] || ( echo "/lib/modules/5.15.147/videobuf2-dma-contig.ko not exist!" | tee -a /var/log/messages && exit 1 ) || exit 1
# [ -f /lib/modules/5.15.147/vin_io.ko               ] || ( echo "/lib/modules/5.15.147/vin_io.ko not exist!"               | tee -a /var/log/messages && exit 1 ) || exit 1
# [ -f /lib/modules/5.15.147/fslidar_mipi.ko         ] || ( echo "/lib/modules/5.15.147/fslidar_mipi.ko not exist!"         | tee -a /var/log/messages && exit 1 ) || exit 1
# [ -f /lib/modules/5.15.147/vin_v4l2.ko             ] || ( echo "/lib/modules/5.15.147/vin_v4l2.ko not exist!"             | tee -a /var/log/messages && exit 1 ) || exit 1

# sleep 5

killall soc_example

# mod_videobuf2=$(lsmod | grep videobuf2_dma_contig | awk -F ' ' '{print $1}')
# mod_vin_io=$(lsmod    | grep vin_io               | awk -F ' ' '{print $1}')
# mod_fslidar=$(lsmod   | grep fslidar_mipi         | awk -F ' ' '{print $1}')
# mod_vin_v4l2=$(lsmod  | grep vin_v4l2             | awk -F ' ' '{print $1}')


# if [ -z "${mod_videobuf2}" ] ;  then
#   if [ -f "${UPDATE_DIR}/lib/modules/videobuf2-dma-contig.ko" ] ; then
#        echo "load ${UPDATE_DIR} videobuf2_dma_contig" | tee -a /var/log/messages
#        insmod ${UPDATE_DIR}/lib/modules/videobuf2-dma-contig.ko
#   else
#        echo "load videobuf2_dma_contig" | tee -a /var/log/messages
#        insmod /lib/modules/5.15.147/videobuf2-dma-contig.ko
#   fi
# fi

# if [ -z "${mod_vin_io}" ] ;  then
#   if [ -f "${UPDATE_DIR}/lib/modules/vin_io.ko" ] ; then
#        echo "load ${UPDATE_DIR} vin_io" | tee -a /var/log/messages
#        insmod ${UPDATE_DIR}/lib/modules/vin_io.ko
#   else
#        echo "load vin_io" | tee -a /var/log/messages
#        insmod /lib/modules/5.15.147/vin_io.ko
#   fi
# fi

# if [ -z "${mod_fslidar}" ] ;  then
#   if [ -f "${UPDATE_DIR}/lib/modules/fslidar_mipi.ko" ] ; then
#        echo "load ${UPDATE_DIR} fslidar_mipi" | tee -a /var/log/messages
#        insmod ${UPDATE_DIR}/lib/modules/fslidar_mipi.ko
#   else
#        echo "load fslidar_mipi" | tee -a /var/log/messages
#        insmod /lib/modules/5.15.147/fslidar_mipi.ko
#   fi
# fi

# if [ -z "${mod_vin_v4l2}" ] ;  then
#   if [ -f "${UPDATE_DIR}/lib/modules/vin_v4l2.ko" ] ; then
#        echo "load ${UPDATE_DIR} vin_v4l2" | tee -a /var/log/messages
#        insmod ${UPDATE_DIR}/lib/modules/vin_v4l2.ko
#   else
#        echo "load vin_v4l2" | tee -a /var/log/messages
#        insmod /lib/modules/5.15.147/vin_v4l2.ko
#   fi
# fi

if [ -f "${UPDATE_DIR}/bin/soc_example" ] ; then
	echo "run ${UPDATE_DIR} lidar app" | tee -a /var/log/messages
	RUN_DIR=${UPDATE_DIR}
else
	echo "run ${ROOT_DIR} lidar app" | tee -a /var/log/messages
	RUN_DIR=${ROOT_DIR}
fi

cd ${RUN_DIR} && ./bin/soc_example

echo "oosp! Lidar app exit!" | tee -a /var/log/messages

exit 1
