#!/bin/bash

source ../devel/setup.bash

function usage {
    echo "usage: mavparam.sh command [filename]"
    echo "Options:"
    echo "    command       Either dump, load, or help"
    echo "    filename      Default ~/Documents/apm_stack_px4.params"
    exit
}

if [ "$2" = "" ]; then
    filename=$HOME/Documents/apm_stack_px4.params
else
    filename=$(readlink -f $2)
fi


if [ $# -eq 0 ] || [ "$1" = "help" ]; then
    usage
elif [ "$1" = "dump" ]; then
    # Dump all parameters from FCU
    roscd mavros
    ./scripts/mavparam dump -mp -f $filename 
    exit
elif [ "$1" = "load" ]; then
    # Load parameters from file
    roscd mavros
    ./scripts/mavparam load -mp $filename
    exit
else
   usage
fi 
