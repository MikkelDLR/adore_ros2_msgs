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
# *  Daniel Heß
# *  Thomas Lobig
# ********************************************************************************
bool is_valid
adore_ros2_msgs/LinearPiecewiseFunction3d centerline                  #s->x,y,z
adore_ros2_msgs/LinearPiecewiseFunction2d center_normal               #s->nx,ny
adore_ros2_msgs/LinearPiecewiseFunction1d left_distance               #s->dl
adore_ros2_msgs/LinearPiecewiseFunction1d right_distance              #s->dr
adore_ros2_msgs/LinearPiecewiseFunction1d center_curvature            #s->kappa
adore_ros2_msgs/LinearPiecewiseFunction1d center_curvature_derivative #s->dkappa/ds
adore_ros2_msgs/LinearPiecewiseFunction1d navigation_cost             #s->c
adore_ros2_msgs/LinearPiecewiseFunction1d speed_limit                 #s->v
adore_ros2_msgs/LinearPiecewiseFunction1d left_indicator_hint         #s->indicator left
adore_ros2_msgs/LinearPiecewiseFunction1d right_indicator_hint        #s->indicator right

#speedLimit
#speedRecommendation (has -> bool, get -> double)
#get Heading -> double
#get Curvature
