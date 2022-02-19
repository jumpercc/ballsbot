#!/bin/bash

if [ -z ${LOCAL_PORT+x} ]; then
  MY_IP=$(ifconfig | grep -P '\binet\b' | grep -vF 'inet 127.0.0.1' | head -1 | awk '{print $2}')
else
  MY_IP=127.0.0.1
fi

jupyter lab --no-browser --notebook-dir=/home/ballsbot/notebooks --ip=$MY_IP
