#!/bin/bash
driver/rplidar_sdk/sdk/output/Darwin/Release/ultra_simple > vis/data.csv 
cd ./vis
python3 plot.py
