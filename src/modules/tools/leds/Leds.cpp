/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "Leds.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "LedsPublicAccess.h"
#include "MRI_Hooks.h"

#include <algorithm>

#define    command_subcode_checksum     CHECKSUM("subcode")
#define    led_count_checksum           CHECKSUM("led_count")
#define    output_pin_checksum          CHECKSUM("output_pin")
#define    output_on_command_checksum   CHECKSUM("output_on_command")
#define    output_off_command_checksum  CHECKSUM("output_off_command")
#define    input_on_command_checksum    CHECKSUM("input_on_command")
#define    input_off_command_checksum   CHECKSUM("input_off_command")

/* Systick Register address, refer datasheet for more info */
#define STCTRL      (*( ( volatile unsigned long *) 0xE000E010 ))
#define STRELOAD    (*( ( volatile unsigned long *) 0xE000E014 ))
#define STCURR      (*( ( volatile unsigned long *) 0xE000E018 ))  

/*******STCTRL bits*******/
#define SBIT_ENABLE     0
#define SBIT_TICKINT    1
#define SBIT_CLKSOURCE  2
#define SBIT_COUNT      16

Leds::Leds() {}

Leds::Leds(uint16_t name)
{
    this->name_checksum = name;
}

// set the pin to the fail safe value on halt
void Leds::on_halt(void *arg)
{
    if(arg == nullptr) {
		this->digital_pin->set(false);
    }
}

void Leds::on_module_loaded()
{
	// save timings in timer CPU ticks
	tres = (SystemCoreClock) * 0.0005F;
	t1h = (SystemCoreClock) * (0.0000009F - 0.0000002F);
	t1l = (SystemCoreClock) * (0.00000035F - 0.00000015F);
	t0h = (SystemCoreClock) * (0.00000035F - 0.00000015F);
	t0l = (SystemCoreClock) * (0.0000009F - 0.0000002F);	

    this->leds_changed = false;

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);


    // Settings
    this->on_config_reload(this);
}

// Get config
void Leds::on_config_reload(void *argument)
{	
    this->subcode = THEKERNEL->config->value(leds_checksum, this->name_checksum, command_subcode_checksum )->by_default(0)->as_number();
    this->led_count = THEKERNEL->config->value(leds_checksum, this->name_checksum, led_count_checksum )->by_default(8)->as_number();
	
    std::string input_on_command = THEKERNEL->config->value(leds_checksum, this->name_checksum, input_on_command_checksum )->by_default("")->as_string();
    std::string input_off_command = THEKERNEL->config->value(leds_checksum, this->name_checksum, input_off_command_checksum )->by_default("")->as_string();

    this->digital_pin= new Pin();
    this->digital_pin->from_string(THEKERNEL->config->value(leds_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
    if(this->digital_pin->connected()) {
        set_low_on_debug(digital_pin->port_number, digital_pin->pin);
    }else{
        delete this->digital_pin;
        this->digital_pin= nullptr;
    }
	
    // Set the on/off command codes, Use GCode to do the parsing
    input_on_command_letter = 0;
    input_off_command_letter = 0;

    if(!input_on_command.empty()) {
        Gcode gc(input_on_command, NULL);
        if(gc.has_g) {
            input_on_command_letter = 'G';
            input_on_command_code = gc.g;
        } else if(gc.has_m) {
            input_on_command_letter = 'M';
            input_on_command_code = gc.m;
        }
    }
    if(!input_off_command.empty()) {
        Gcode gc(input_off_command, NULL);
        if(gc.has_g) {
            input_off_command_letter = 'G';
            input_off_command_code = gc.g;
        } else if(gc.has_m) {
            input_off_command_letter = 'M';
            input_off_command_code = gc.m;
        }
    }
	
    this->digital_pin->set(this->leds_state);
	this->brg1 = this-> brg2 = 0x000000;
    set_colors(brg1, brg2); // turn off leds by default.	
}

bool Leds::match_input_on_gcode(const Gcode *gcode) const
{
    bool b= ((input_on_command_letter == 'M' && gcode->has_m && gcode->m == input_on_command_code) ||
            (input_on_command_letter == 'G' && gcode->has_g && gcode->g == input_on_command_code));

    return (b && gcode->subcode == this->subcode);
}

bool Leds::match_input_off_gcode(const Gcode *gcode) const
{
    bool b= ((input_off_command_letter == 'M' && gcode->has_m && gcode->m == input_off_command_code) ||
            (input_off_command_letter == 'G' && gcode->has_g && gcode->g == input_off_command_code));
    return (b && gcode->subcode == this->subcode);
}

uint8_t Leds::reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void Leds::on_gcode_received(void *argument)
{
	uint8_t c = 0;	
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Add the gcode to the queue ourselves if we need it
    if (!(match_input_on_gcode(gcode) || match_input_off_gcode(gcode))) {
        return;
    }

    // we need to sync this with the queue, so we need to wait for queue to empty, however due to certain slicers
    // issuing redundant swicth on calls regularly we need to optimize by making sure the value is actually changing
    // hence we need to do the wait for queue in each case rather than just once at the start
    if(match_input_on_gcode(gcode)) {
        auto args = gcode->get_args();
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		
		if(args.find('R') != args.end()) {
			r = reverse((uint8_t)args['R']);
		}
		if(args.find('A') != args.end()) {
			g = reverse((uint8_t)args['A']);
		}
		if(args.find('B') != args.end()) {
			b = reverse((uint8_t)args['B']);			
		}
		if(args.find('C') != args.end()) {
			c = (uint8_t)args['C'];
		}
        uint32_t brg = (b << 16) | (r << 8) | (g << 0);		
        switch(c) {
            case 1:
				this->brg1 = brg;
            break;		  
            case 2:
				this->brg2 = brg;
            break;
            default:
			    this->brg1 = this->brg2 = brg;
        }
        // drain queue
        THEKERNEL->conveyor->wait_for_idle();
        this->leds_state = true;
        set_colors(brg1, brg2);
    } else if(match_input_off_gcode(gcode)) {
        auto args = gcode->get_args();		
		if(args.find('C') != args.end()) {
			c = (uint8_t)args['C'];
		}
        switch(c) {
            case 1:
				this->brg1 = 0;
            break;		  
            case 2:
				this->brg2 = 0;
            break;
            default:
			    this->brg1 = this->brg2 = 0;
        }
        // drain queue
        THEKERNEL->conveyor->wait_for_idle();
        this->leds_state = false;
        set_colors(this->brg1, this->brg2);
    }
}

void Leds::set_colors(uint32_t brg1, uint32_t brg2)
{	
    int32_t t0 = 0;
	int16_t led_half_count = led_count/2;
    // Configure SYSTICK
	STCURR = 0xffffff;// this resets to zero! otherwise it will keep value between enable/disable
    STRELOAD = 0xffffff;    // Reload value for largest tick possible
	STCTRL = (1<<SBIT_ENABLE) | (1<<SBIT_CLKSOURCE);
	while(STCURR == 0) {};// wait it flips!
	
    this->digital_pin->set(false);
	// Delay 500uS
	t0 = STCURR; while((t0 - STCURR) < tres) {}	
	__disable_irq();
	//
	for(uint16_t j = 0; j < led_half_count; j++) {
		uint32_t color = brg1;
	    for(uint8_t i = 0; i < 24; i++) {
		    if (color & 0x1) {
                this->digital_pin->set(true);				
    	        t0 = STCURR; while((t0 - STCURR) < t1h) {}
                this->digital_pin->set(false);								
	            t0 = STCURR; while((t0 - STCURR) < t1l) {}						
	    	} else {
                this->digital_pin->set(true);												
	            t0 = STCURR; while((t0 - STCURR) < t0h) {}
                this->digital_pin->set(false);																
	            t0 = STCURR; while((t0 - STCURR) < t0l) {}									
		    }
		    color >>= 1; 
	    }
	}
	//
	for(uint16_t j = led_half_count; j < led_count; j++) {
		uint32_t color = brg2;
	    for(uint8_t i = 0; i < 24; i++) {
		    if (color & 0x1) {
                this->digital_pin->set(true);				
    	        t0 = STCURR; while((t0 - STCURR) < t1h) {}
                this->digital_pin->set(false);								
	            t0 = STCURR; while((t0 - STCURR) < t1l) {}						
	    	} else {
                this->digital_pin->set(true);												
	            t0 = STCURR; while((t0 - STCURR) < t0h) {}
                this->digital_pin->set(false);																
	            t0 = STCURR; while((t0 - STCURR) < t0l) {}									
		    }
		    color >>= 1; 
	    }
	}	
    //
    this->digital_pin->set(false);													
	__enable_irq();	
	STCTRL = 0; //Stop systick	
}
