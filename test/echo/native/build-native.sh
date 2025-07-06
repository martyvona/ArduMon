#!/bin/bash

src=`ls *.cpp`
g++ -I../../../src -o ${src%.*} $src --std=c++11

