#!/bin/bash

echo "buidlng for native"

OPTS=--std=c++11

# -Wstringop-overflow=0 suppresses a spurious warning on some versions of g++ with -O3 
# https://stackoverflow.com/a/75191691
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=106757
if ! g++ --version | grep clang > /dev/null; then OPTS="$OPTS -Wstringop-overflow=0"; fi

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DBG=
if [[ $# -gt 0 && $1 == "-d" ]]; then
DBG=" (debug)"
OPTS="$OPTS -g"
else
OPTS="$OPTS -O3"
fi

echo "building native ardumon_server${DBG}..."
g++ $OPTS -I${script_dir}/../../../src -o ardumon_server demo.cpp || exit $?

echo "building native ardumon_client${DBG}..."
g++ $OPTS -I${script_dir}/../../../src -o ardumon_client -DDEMO_CLIENT demo.cpp || exit $?

