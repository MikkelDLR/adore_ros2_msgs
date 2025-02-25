# *******************************************************************************
# * Copyright (C) 2017-2023 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Daniel Heß
# *  Matthias Nichting
# ********************************************************************************

VehicleStateDynamic motion_state
Trajectory predicted_trajectory
geometry_msgs/Point goal_point
Route route
TrafficClassification classification
int32 tracking_id

PhysicalVehicleParameters physical_parameters

float64 dimensions_variance # variance of dimensions of shape
uint8 classification_certainty	# confidence of classification 0..100
uint8 existance_certainty # confidence that traffic participant exists 0..100
bool left_indicator_on # indicator in yaw direction on the left side
uint8 left_indicator_certainty # confidence of detection of left indicator state 0..100
bool right_indicator_on # indicator in yaw direction on the right side
uint8 right_indicator_certainty # confidence of detection of right indicator state 0..100
bool brake_light_on # state of the brake lights
uint8 brake_light_certainty # confidence of detection of brake light state 0..100
bool low_beam_on # state of the low beam
uint8 low_beam_certainty # confidence of detection of low beam state 0..100
bool high_beam_on # state of the high beam
uint8 high_beam_certainty # confidence of detection of high beam state 0..100
uint64 v2x_station_id # etsi Vehicle to X id
uint8 v2x_station_id_certainty # confidence of association of v2xStationID

# certainty scaling
uint8 CERTAINTY_MAX = 100