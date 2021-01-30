#!/bin/bash

MY_IP=$(ifconfig | grep -P '\binet\b' | grep -vF 'inet 127.0.0.1' | head -1 | awk '{print $2}');

jupyter notebook --notebook-dir=/home/ballsbot/notebooks --no-browser --ip=$MY_IP --config=/home/ballsbot/.jupyter/jupyter_notebook_config.py
