// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// include the header of your new driver here similar to default_imu.h
#include "default_imu.h"

// now you can create a config constant that you can use in lino_base_config.h
#ifdef USE_QMI8658_IMU
    #define IMU QMI8658IMU
#endif

#endif
