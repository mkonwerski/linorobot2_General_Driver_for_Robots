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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

// include IMU base interface
#include "imu_interface.h"

// include sensor API headers
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "QMI8658.h"

class QMI8658IMU : public IMUInterface
{
private:
    QMI8658 qmi8658_;

    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;

public:
    QMI8658IMU()
    {
    }

    bool startSensor() override
    {
        if (qmi8658_.begin() == 0)
        {
            return false;
        }
        return true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        float ac[3];
        qmi8658_.read_acc(ac);
        accel_.x = ac[0];
        accel_.y = ac[1];
        accel_.z = ac[2];
        return accel_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        float gy[3];
        qmi8658_.read_gyro(gy);
        gyro_.x = gy[0];
        gyro_.y = gy[1];
        gyro_.z = gy[2];
        return gyro_;
    }
};

#endif