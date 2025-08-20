#!/bin/bash

echo "buidlng for native"

# -Wstringop-overflow=0 suppresses a spurious warning on some versions of g++ with -O3 
# https://stackoverflow.com/a/75191691
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=106757
OPTS=
if ! g++ --version | grep clang > /dev/null; then OPTS=-Wstringop-overflow=0; fi

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "building native text/binary server..."
g++ -O3 -I${script_dir}/../../src -o demo_native demo.cpp --std=c++11 $OPTS

echo "building native binary client..."
g++ -O3 -I${script_dir}/../../src -o demo_client_native -DBINARY_CLIENT demo.cpp --std=c++11 $OPTS

