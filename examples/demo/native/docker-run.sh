#!/bin/bash

docker build -t ardumon-demo .
docker run -it -v $PWD/../..:/app ardumon-demo
#cd /app/demo/native
