#!/bin/bash

CID=`docker ps | grep ardumon-demo | cut -d' ' -f1`
docker exec -it $CID bash
