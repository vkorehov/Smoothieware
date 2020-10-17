/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once

#include <stdint.h>
#include <array>
#include <bitset>
#include <functional>
#include <atomic>

#include "TSRingBuffer.h"

class WS2812{
    public:
        WS2812();
        ~WS2812();
        void set_color( uint32_t color );
        void set_led_count( uint32_t count );
        
        void start();
        void stop();
        void tick(void);

        static WS2812 *getInstance() { return instance; }

    private:
        uint8_t green;
        uint8_t red;
        uint8_t blue;
                
        uint32_t led_count;
        static WS2812 *instance;
};
