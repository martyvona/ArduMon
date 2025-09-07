#!/bin/bash

CID=`docker ps | grep ardumon-valgrind | cut -d' ' -f1`
docker exec -it $CID bash
