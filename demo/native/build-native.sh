#!/bin/bash

src=`ls *.cpp`
g++ -O3 -I../../src -o ${src%.*}_native $src --std=c++11

