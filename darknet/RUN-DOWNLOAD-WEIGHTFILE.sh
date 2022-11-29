#!/bin/bash

echo "Create folder to download weights file..."
cmd="mkdir weights"
echo "$cmd"
$cmd


echo "Download wight files..."

## yolov2-tiny
# wget -P ./weights/ https://pjreddie.com/media/files/yolov2-tiny.weights

## yolov2
# wget -P ./weights/ https://pjreddie.com/media/files/yolov2.weights

## yolov3-tiny
# wget -P ./weights/ https://pjreddie.com/media/files/yolov3-tiny.weights

## yolov3
# wget -P ./weights/ https://pjreddie.com/media/files/yolov3.weights

## yolov4-tiny
wget -P ./weights/ https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights

## yolov4
wget -P ./weights/ https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
