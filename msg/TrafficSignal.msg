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
# *   Marko Mizdrak
# ********************************************************************************
float64 x               # x coordinate
float64 y               # y coordinate
int64 signal_group_id   # unique identifier for the traffic light's signal group
int32 state             # traffic light state as an enum

# Enum values for the traffic light state
int32 RED = 0
int32 YELLOW = 1
int32 GREEN = 2
int32 UNKNOWN = 3
