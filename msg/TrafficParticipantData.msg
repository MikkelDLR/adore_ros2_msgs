# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *   Giovanni Lucente
# ********************************************************************************
geometry_msgs/Pose2D pos                            # state of the i-th vehicle
float64 l                                           # length of the i-th vehicle */
float64 w                                           # width of the i-th vehicle */
float64 v                                           # speed */
float64 v_long                                      # longitudinal speed of the i-th vehicle */
float64 v_lat                                       # lateral speed of the i-th vehicle */
float64 a_long                                      # longitudinal acceleration of the i-th vehicle */
float64 beta                                        # slip angle */
int64 tracking_id                                    # id */
Lane[] center_lanes                              # paths of the i-th vehicle keeping the lane */
Lane[] left_lanes                       # paths of the i-th vehicle changing lane on the left */
Lane[] right_lanes                      # paths of the i-th vehicle changing lane on the right */