#!/bin/bash

src=`ls *.cpp`
# -Wstringop-overflow=0 suppresses a spurious warning on some versions of g++ with -O3 
# https://stackoverflow.com/a/75191691
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=106757
OPTS=
if ! g++ --version | grep clang > /dev/null; then OPTS=-Wstringop-overflow=0; fi
g++ -O3 -I../../src -o ${src%.*}_native $src --std=c++11 $OPTS

