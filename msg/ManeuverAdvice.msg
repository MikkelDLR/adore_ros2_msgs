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
# *   Mikkel Skov Maarssoe
# *   Giovanni Lucente
# ********************************************************************************

# --- ASN.1 Definition ---------------------------------------------------------
# ManeuverType ::= ENUMERATED {
# 	freeTravel           (0),
# 	followRoadUser       (1),
# 	crossIntersection    (2),
# 	yieldAtIntersection  (3),
# 	overtake             (4),
# 	pullOver             (5),
# 	yieldAtObstacle      (6),
# 	maneuverTypeUnknown  (15)
# }
# ------------------------------------------------------------------------------

uint8 value
uint8 FREE_TRAVEL = 0
uint8 FOLLOW_ROAD_USER = 1
uint8 CROSS_INTERSECTION = 2
uint8 YIELD_AT_INTERSECTION = 3
uint8 OVERTAKE = 4
uint8 PULL_OVER = 5
uint8 YIELD_AT_OBSTACLE = 6
uint8 MANEUVER_TYPE_UNKNOWN = 15

