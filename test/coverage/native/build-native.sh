#!/bin/bash

src=`ls *.cpp`
g++ -I../../../src -o ${src%.*}_native $src --std=c++11

