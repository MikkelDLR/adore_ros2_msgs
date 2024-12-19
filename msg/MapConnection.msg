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
uint64 from_id
uint64 to_id
uint8 connection_type
float64 weight

# connection types
uint8 END_TO_START=0
uint8 END_TO_END=1
uint8 START_TO_START=2
uint8 START_TO_END=3
