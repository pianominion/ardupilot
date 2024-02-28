/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_OpenMV {
public:
    AP_OpenMV(void);

    /* Do not allow copies */
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV &operator=(const AP_OpenMV&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager& serial_manager);
    //update flight control mode.The control mode is vehicle type specific
    bool update(void);

    uint8_t cx;
    uint8_t cy;
    uint32_t last_frame_ms;
    

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    
    uint8_t _step;
    uint8_t _cx_temp;
    uint8_t _cy_temp;
};
