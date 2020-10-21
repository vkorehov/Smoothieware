/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Pin.h"

#include <math.h>
#include <string>

class Gcode;
class StreamOutput;

class Leds : public Module {
    public:
        Leds();
        Leds(uint16_t name);

        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);
        void on_halt(void *arg);

    private:
        bool match_input_on_gcode(const Gcode* gcode) const;
        bool match_input_off_gcode(const Gcode* gcode) const;

		uint8_t reverse(uint8_t b);		
        void set_colors(uint32_t brg);
		uint32_t      led_count;
        Pin           *digital_pin;
        struct {
			uint32_t  tres;
			uint32_t  t1h;
			uint32_t  t1l;
			uint32_t  t0h;
			uint32_t  t0l;
			
            uint16_t  name_checksum:16;
            uint16_t  input_on_command_code:16;
            uint16_t  input_off_command_code:16;
            char      input_on_command_letter:8;
            char      input_off_command_letter:8;
            uint8_t   subcode:4;
            bool      leds_changed:1;
            bool      leds_state:1;
        };
};
