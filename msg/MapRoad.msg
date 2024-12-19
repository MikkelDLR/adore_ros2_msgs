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
uint64 road_id
string name
bool one_way 
uint64 id
uint8 category 
MapLane[] lanes

uint8 UNKNOWN=0
uint8 RURAL=1
uint8 MOTORWAY=2
uint8 TOWN=3
uint8 LOW_SPEED=4
uint8 PEDESTRIAN=5
uint8 BICYCLE=6
