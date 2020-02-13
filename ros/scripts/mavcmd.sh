MAV_CMD_DO_SET_MODE=8

MAV_CMD_DO_SET_MODE=176
MODE=$MAV_MODE_FLAG_GUIDED_ENABLED
CUSTOM_MODE=0
CUSTOM_SUBMODE=0

../src/mavros/mavros/scripts/mavcmd -v int $MAV_CMD_DO_SET_MODE \
                                           $MODE \
                                           $CUSTOM_MODE \
                                           $CUSTOM_SUBMODE \
                                           0 0 0 0 0
