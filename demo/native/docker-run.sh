#!/bin/bash

docker build -t ardumon-valgrind .
docker run -it -v $PWD/../..:/app ardumon-valgrind
#cd /app/demo/native
