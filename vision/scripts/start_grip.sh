#!/bin/bash

env LD_LIBRARY_PATH=~/vision/grip:$LD_LIBRARY_PATH java -jar ~/vision/grip/grip.jar ~/code/grip/project.grip &
