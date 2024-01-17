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
# ********************************************************************************
BorderID border_id
# array of points
geometry_msgs/Point[] points

bool has_left_id

BorderID left_id

int16 border_type

int32 data_base_id
int32 parrent_id
int32 predecessor_id
int32 successor_id