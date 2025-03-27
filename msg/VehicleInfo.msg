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
# * Marko Mizdrak
# * Mikkel Skov Maarssø
# ********************************************************************************
# VehicleInfo.msg
int32   id
uint64  v2x_station_id
GearState gear_state
float64 wheel_speed
bool left_indicator_on
bool right_indicator_on
bool automatic_steering_on
bool automatic_acceleration_on
bool automatic_acceleration_active
bool clearance
PhysicalVehicleParameters physical_parameters
float64 localization_error # standard deviation error
float64 localization_frequency
float64 traffic_participant_detection_frequency
