#message for fm  passenger requests
uint32 command_id
uint8 command 

bool enable
int32 arg_int
float32 arg_float

# command ids
uint8 FOLLOW_LANE=2
uint8 CHANGE_LANE_LEFT=3
uint8 CHANGE_LANE_RIGHT=4
uint8 STOP_AND_PARK=9
uint8 DRIVE_BACK_TO_START=16
uint8 RESUME_RIDE=19
uint8 KEEP_MORE_DISTANCE=20