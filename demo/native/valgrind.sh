#!/bin/bash

./build-native.sh -d
valgrind --leak-check=full --show-leak-kinds=all ./ardumon_server foo
