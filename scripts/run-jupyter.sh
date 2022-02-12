#!/bin/bash

MY_IP=$(ifconfig | grep -P '\binet\b' | grep -vF 'inet 127.0.0.1' | head -1 | awk '{print $2}');

jupyter lab --no-browser --notebook-dir=/home/ballsbot/notebooks --ip=$MY_IP
