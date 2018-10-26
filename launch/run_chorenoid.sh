#!/bin/bash

### choose choreonoid binary
choreonoid_exe='choreonoid'

cnoid_proj=""
if [ "$(echo $1 | grep \.cnoid$ | wc -l)" == 1 ]; then
    cnoid_proj=$1
fi
start_sim=""
enable_const=""
add_objects=""
rtm_args=()
latch=0

export CNOID_CUSTOMIZER_PATH=$(rospack find choreonoid_plugins)
export CUSTOMIZER_CONF_PATH=$(rospack find choreonoid_plugins)/config/trampoline.yaml

if [ -z "$cnoid_proj" ]; then
    (cd /tmp; $choreonoid_exe)
else
    (cd /tmp; $choreonoid_exe $cnoid_proj)
    ## for using gdb
    #(cd /tmp; gdb -ex run --args $choreonoid_exe $enable_const $add_objects $cnoid_proj $start_sim)
fi
