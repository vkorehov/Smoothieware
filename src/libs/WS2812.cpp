/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "WS2812.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SlowTicker.h"
#include "StepTicker.h"
#include "libs/Hook.h"
#include "modules/robot/Conveyor.h"
#include "Gcode.h"

#include <mri.h>


WS2812 *WS2812::instance;

WS2812::WS2812()
{
    instance = this; // setup the Singleton instance of the stepticker

    // Configure the timer
    float period = floorf((SystemCoreClock / 4.0F) / 20000000.0F); // SystemCoreClock/4 = Timer increments in a second
    
    LPC_TIM3->MR0 = period;       // 
    LPC_TIM3->MCR = 3;            // Match on MR0, reset on MR0
    LPC_TIM3->TCR = 0;            // Enable interrupt

    LPC_SC->PCONP |= (1 << 23);      // Timer3 ON    
}

WS2812::~WS2812()
{
}

//50ns tick pin 4.28 timer2 channel0 (MAT2[0]):
// PINSEL9 bit 25 = 1, bit 24 = 0
void WS2812::tick(void)
{

}

//called when everything is setup and interrupts can start
void WS2812::start()
{
    NVIC_EnableIRQ(TIMER3_IRQn);     // Enable interrupt handler
}

void WS2812::stop()
{
    NVIC_DisableIRQ(TIMER3_IRQn);     // Disable interrupt handler
}

extern "C" void TIMER3_IRQHandler (void)
{
    LPC_TIM3->IR |= 1 << 0;
    WS2812::getInstance()->tick();
}
