#!/bin/bash

env LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH mjpg_streamer -o "output_http.so -w ./www -p 1181" -i "input_uvc.so -f 30 -r 640x480 -y -" &
