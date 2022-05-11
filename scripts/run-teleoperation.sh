#!/bin/bash

cd ~/projects/ballsbot/python/scripts/

python3 ./teleoperation-bot.py --ip=$1 "--server-config-dir=/home/ballsbot/web_server_config_$1"
