#include "stm32l476xx.h"
#include "SysTick.h"
#include "LED.h"
#include "lcd.h"
#include "UART.h"
#include "Music_Player.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define FREQ_CORRECTION_FACTOR 1.547
#define TIME_CORRECTION_FACTOR 3.79

volatile double angle, frequency = 0;
volatile double bpm = 60;

void System_Clock_Init(void){
	
	RCC->CR |= RCC_CR_MSION; 
	
	// Select MSI as the clock source of System Clock
	RCC->CFGR &= ~RCC_CFGR_SW; 
	
	// Wait until MSI is ready
	while ((RCC->CR & RCC_CR_MSIRDY) == 0); 	
	
	// MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1). 
	RCC->CR &= ~RCC_CR_MSIRANGE; 
	RCC->CR |= RCC_CR_MSIRANGE_10;  // Select MSI 32 MHz, we want to select 11, but this seems to be broken
 
	// The MSIRGSEL bit in RCC-CR select which MSIRANGE is used. 
	// If MSIRGSEL is 0, the MSIRANGE in RCC_CSR is used to select the MSI clock range.  (This is the default)
	// If MSIRGSEL is 1, the MSIRANGE in RCC_CR is used. 
	RCC->CR |= RCC_CR_MSIRGSEL; 
	
	// Enable MSI and wait until it's ready	
	while ((RCC->CR & RCC_CR_MSIRDY) == 0); 		
}



void DAC_Channel2_Init(void){
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
	
	DAC->CR &= ~( DAC_CR_EN1 | DAC_CR_EN2);
	
	DAC->MCR &= ~(7U<<16);
	
	//Enable trigger for DAC channel 2
	DAC->CR |= DAC_CR_TEN2;

	DAC->CR &= ~DAC_CR_TSEL2;
	
	DAC->CR |= DAC_CR_TSEL2_0; //101 is TIM4
	DAC->CR |= DAC_CR_TSEL2_2; 
	
	
	//Enable DAC Channel 2
	DAC->CR |= DAC_CR_EN2;
	
	//Enable GPIO port A Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	//Set I/O mode as analog
	GPIOA->MODER |= 3U<<(2*5);
	
}

void TIM4_Init() {
	  // Enable the Timer 4 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	  
	  // Clears so bits and selects counting directions 
	  TIM4->CR1 &= ~TIM_CR1_CMS;
	  TIM4->CR1 &= ~TIM_CR1_DIR;
	
	  // Clears master mode selection bits
	  TIM4->CR2 &= ~TIM_CR2_MMS;
	
	  // Select 100 as TRGO
	  TIM4->CR2 |= TIM_CR2_MMS_2;
	
	  // clear mode bits
	  TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	
	  // set mode to 0110
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

    TIM4->PSC = 5; 
    TIM4->ARR = 6;
    TIM4->CCR1 = 1;
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
		
		TIM4->DIER |= TIM_DIER_UIE;

	  // Enables IRQ in NVIC
	  NVIC_SetPriority(TIM4_IRQn, 1);
	  NVIC_EnableIRQ(TIM4_IRQn);

}

void TIM4_IRQHandler() {
	  double result;
	  double step;
		uint32_t output = 0;
    if ((TIM4->SR & TIM_SR_CC1IF) != 0) {
		  result = angle * M_PI / 180.0f; //put angle in radians
			
			output = (uint32_t)((sin(result) + 1)* 2048.0); // Theoretically if you make a super fast sin talbe this should work 
			
			while ((DAC->SR & DAC_SR_BWST2) != 0);
			
			DAC->DHR12R2 = output;
			
			step = 360.0f * frequency / 20000.0f;
			
			angle += step;  
		
			angle = angle > 360.0f ? 0.0 : angle;
			
			TIM4->SR &= ~TIM_SR_CC1IF;
		}
		
		if ((TIM4->SR & TIM_SR_UIF) != 0) {
		    TIM4->SR &= ~TIM_SR_UIF;
		}
}

double Convert_Freq(double input_freq) {
    return FREQ_CORRECTION_FACTOR * input_freq;
}

uint32_t Convert_Time(uint32_t time) {
    return (uint32_t)(TIME_CORRECTION_FACTOR * time);
}

void Play_Freq(double input_freq, uint32_t time) {
    frequency = Convert_Freq(input_freq);
	  delay((Convert_Time(time)));
	  
}

double Octavize_Freq(double freq, uint16_t octave) {
    if (octave == 4) {
		    return freq * 16.0f;
		} else if (octave == 5) {
			  return freq * 32.0f;
		} else if (octave == 2) {
		    return freq * 4.0f;
		} else if (octave == 3) {
		    return freq * 8.0f;
		} else if (octave == 0) {
		    return freq;
		} else if (octave == 1) {
		    return freq * 2.0f;
		} else if (octave == 6) {
		    return freq * 64.0f;
		}
}

double Note_To_Freq(char note, bool sharp, bool flat, uint16_t octave) {
    switch(note) {

    case 'A':
        if (sharp) {
					A_SHARP:
            return Octavize_Freq(29.135, octave);
        } else if (flat) {
					A_FLAT:
			      return Octavize_Freq(25.295, octave);
			  } else {
					A_NATURAL:
			      return Octavize_Freq(27.3, octave);
			  }
		
		case 'B':
        if (sharp) {
		      B_SHARP:
            return Octavize_Freq(16.352, octave);
        } else if (flat) {
		        goto A_SHARP;
		    } else {
		      B_NATURAL:
		        return Octavize_Freq(30.868, octave);
        }

		case 'C':
        if (sharp) {
		      C_SHARP:
            return Octavize_Freq(17.324, octave);
        } else if (flat) {
		        goto B_NATURAL;
		    } else {
		        goto B_SHARP;
        }

		case 'D':
        if (sharp) {
		      D_SHARP:
            return Octavize_Freq(19.445, octave);
        } else if (flat) {
		        goto C_SHARP;
		    } else {
					D_NATURAL:
					  return Octavize_Freq(18.0, octave);
        }

		case 'E':
        if (sharp) {
		      E_SHARP:
            return Octavize_Freq(21.827, octave);
        } else if (flat) {
		        goto D_SHARP;
		    } else {
		      E_NATURAL:
		        return Octavize_Freq(20.602, octave);
        }

		case 'F':
        if (sharp) {
		      F_SHARP:
            return Octavize_Freq(23.125, octave);
        } else if (flat) {
		        goto E_NATURAL;
		    } else {
		        goto E_SHARP;
        }
	
		case 'G':
        if (sharp) {
		        goto A_FLAT;
        } else if (flat) {
		        goto F_SHARP;
		    } else {
		      G_NATURAL:
		        return Octavize_Freq(24.0, octave);
        }
				
		case 'R':
			return 1;

    }
}


void Play_Note(char note, bool sharp, bool flat, uint16_t octave, double beats) {
   uint32_t duration = (uint32_t)((beats * (1 / bpm) * 60000.0f)); 
	 uint32_t space = (uint32_t)(.01 * (1 / bpm) * 60000.0f);
	
   Play_Freq(Note_To_Freq(note, sharp, flat, octave), duration);
}



//SONGS (CHAR = NOTES, DOUBLE = FREQ, FLAT = FLAT BOOL, SHARP = SHAPR BOOL)
char hot_cross_char[] = { 'E', 'E', 'E','E','E','E','E','G','C','D','E','F','F','F','F','F','E','E','E','E','E','D','D','E','D','G'
};



//double hot_cross_double[] = { 329.628, 329.628, 329.628, 329.628, 329.628, 329.628, 329.628, 391.995, 261.626, 293.665, 329.628, 349.228, 349.228, 349.228, 349.228, 349.228, 349.228, 329.628, 329.628, 329.628, 329.628, 261.626, 261.626, 329.628, 261.626, 391.995,
//};  
//

double hot_cross_beats[] = { .25,.25,.5,.25,.25,.5,.25,.25,.375,.125,1,.25,.25,.375,.125,.25,.25,.25,.125,.125,.25,.25,.25,.25,.5, .5
};

char HP_char[] = { 'E', 'A', 'C', 'B', 'A', 'E', 'D', 'B',
									 'A', 'C', 'B', 'G', 'B', 'E', 'E', 
									 'A', 'C', 'B', 'A', 'E', 'G', 'F', 'F', 'C',
									 'F', 'E', 'E', 'E', 'C', 'A', 'C',
};

bool HP_sharp[] = {0,0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,1,0,1,
	                 0,0,0,0,0,0,0
};

bool HP_flat[] =  {0,0,0,0,0,0,0,0,
	                 0,0,0,0,1,0,0,
	                 0,0,0,0,0,0,0,0,0,
	                 0,0,1,1,0,0,0
};

uint16_t HP_octave[] ={4,4,5,4,4,5,5,4,
	                     4,5,4,4,4,4,4,
	                     4,5,4,4,5,5,5,5,5,
	                     5,5,5,4,5,4,5
};

double HP_beats[] ={.25,.375,.125,.25,.5,.25,.5,.5,
	                  .375,.125,.25,.5,.25,1,.25,
	                  .375,.125,.25,.5,.25,.5,.25,.5,.25,
	                  .375,.125,.25,.5,.25,1,.25
};

//char G4_char[] = { 'R','C','C','D','E','D','C','D','E','D','C','D','E','G','E','R','C','C','D','E','D','E','D','C','D','E','D','C','D','E','G','E'
//};


//double G4_beats[] ={.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.25,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.125,.25
//};



char taiwan_notes[] = { 'C', 'C', 'E', 'E', 'G', 'G', 'E', 'D', 'E', 'C', 'A', 'G', 'A', 'E', 'A', 'G', 'F', 'G' };
char taiwan_sharps[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
double taiwan_beats[] = { 1, 2.5, 1, 2.5, 1, 1.5, 1, 2.5, 1, 2.5, 0.5, 0.5, 2.5, 1, 2.5, 0.5, 0.5, 2 };
char taiwan_octaves[] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 4, 4, 4 };


int main(void){
		
	int i;
	
	//System Clock Initialization
	System_Clock_Init();
	
	SysTick_Initialize(1000);

	DAC_Channel2_Init();
	
	TIM4_Init();
	
	//LED_Init();
	//Green_LED_On();

	while(1) {	// first row is when divide by freq is 20000 : 3:18:9
 
		//frequency = 402.0f; 
		//Correct_Frequency	(440.0f);
		
//		Green_LED_Toggle();
		//delay(1000);
		
	//	Play_Freq(Note_To_Freq('C', 0, 0, 4), 1000);
	
//		for (i=0;i<27;i++){
//			hcc = hot_cross_char[i];
//			hcd = hot_cross_double[i];
//			hcs = hot_cross_sharp[i];
//			hcf = hot_cross_flat[i];
//			hcb = hot_cross_beats[i];
//			Play_Freq(hcd,hcb*800);
//			delay(1);
//			frequency = 0.1;
//			delay(1);
//			
		//}
    bpm = 60;
  	for (i=0;i<34;i++){
  			Play_Note(HP_char[i],HP_sharp[i],HP_flat[i], HP_octave[i],HP_beats[i]);
  		frequency = 0.1f;
  	}
////bpm = 50;
////		for (i=0;i<31;i++){
////				Play_Note(G4_char[i],0,0, 4,G4_beats[i]);
////		}
    bpm = 100;
    for (i = 0; i < 18; i++) {
		    Play_Note(taiwan_notes[i], taiwan_sharps[i], 0, taiwan_octaves[i], taiwan_beats[i]);
					frequency = 0.1f;
		}
		bpm = 40;
		for (i = 0; i < 27; i++) {
		    Play_Note(hot_cross_char[i], 0, 0, 4, hot_cross_beats[i]);
					frequency = 0.1f;
		}


	//	Play_Note('C',0,0, 4, 0.25);
	//	Play_Note('C',1,0, 4, 0.25);
	//	Play_Note('D',0,0, 5, 0.25);
	//	Play_Note('D',1,0, 4, 0.25);
	//	Play_Note('E',0,0, 4, 0.25);
	//	Play_Note('F',0,0, 4, 0.25); // A and B are fucked up
	//	Play_Note('F',1,0, 4, 0.25);
	//	

		
		// C:  261.626 --> 402.0f
		// C#: 277.183 --> 426
    // D:  293.665 --> 445
    // D#: 311.127 --> 477
    // E:  329.628 --> 513
    // F:  349.228 --> 545
    // F#: 369.994 --> 572
    // G:  391.995 --> 600
    // G#: 415.305 --> 648
    // A:  440.0   --> 675
    // A#: 466.164 --> 715
    // B:  493.883 --> 765
	}
}


		

