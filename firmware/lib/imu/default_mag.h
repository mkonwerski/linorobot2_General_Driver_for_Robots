// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
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

#ifndef DEFAULT_MAG
#define DEFAULT_MAG

//include MAG base interface
#include "mag_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "AK09918.h"

class AK09918MAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        AK09918 magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        AK09918MAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            bool ret;
            ret = magnetometer_.initialize();
            if (ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);

            mag_.x = ax * 0.00000015;
            mag_.y = ay * 0.00000015;
            mag_.z = az * 0.00000015;

            return mag_;
        }
};

class FakeMAG: public MAGInterface
{
    private:
        geometry_msgs__msg__Vector3 mag_;

    public:
        FakeMAG()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            mag_.x = 0.000020;
            return mag_;
        }
};
#endif
