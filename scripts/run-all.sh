#!/bin/bash
set -eo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ -z ${NO_MAKE_ROS_MODULES+x} ]; then
    $DIR/make-ros-modules.sh
    if [ $? -ne 0 ]
    then
      exit 2
    fi
fi

$DIR/run_nodes.sh run_all_1_10.launch > ~/ros.log 2>&1 &
sleep 5s

if [ -z ${LOCAL_PORT+x} ]; then
  MY_IP=$(ifconfig | grep -P '\binet\b' | grep -vF 'inet 127.0.0.1' | head -1 | awk '{print $2}')
else
  MY_IP=127.0.0.1
fi

if [ -z ${TELEOPERATION+x} ]; then
  $DIR/run-jupyter.sh --ip=$MY_IP
else
  $DIR/run-teleoperation.sh $MY_IP
fi

kill %2 2>/dev/null
kill %1 2>/dev/null
