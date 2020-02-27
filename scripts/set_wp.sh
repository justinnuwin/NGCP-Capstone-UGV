#!/bin/bash

source ../ros/devel/setup.bash

function usage {
    echo "usage: set_wp.sh [-h] filename"
    echo "Options:"
    echo "    --help, -h    Show this help text"
    echo "    filename      QGC Waypoint file. See example_wp.qgc in project root"
    exit
}

if [ $# = 0 ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    usage
fi

if [ ! -f $1 ]; then
    echo "$1: No such file"
    exit -1
else
    filename=$(readlink -f $1)
fi

roscd mavros
./scripts/mavwp clear
./scripts/mavwp load $filename

./scripts/mavwp show
./scripts/mavwp setcur 0
