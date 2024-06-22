#!/bin/bash

docker run --rm --name mlmp-compile --volume /home/toms/workspace/robotics/MLMP:/home/mlmp  -it -d  ompl:latest

