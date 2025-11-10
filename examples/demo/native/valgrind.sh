#!/bin/bash

# valgrind runs best in Linux
# use the provided Docker container

./build-native.sh -d
valgrind --leak-check=full --show-leak-kinds=all ./ardumon_server foo
