#!/bin/bash

jupyter lab --no-browser --notebook-dir=/home/ballsbot/notebooks --ip=$1 "$@"
