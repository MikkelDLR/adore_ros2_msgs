# *******************************************************************************
# * Copyright (C) 2017-2025 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *   Malte Hauff
# ********************************************************************************


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