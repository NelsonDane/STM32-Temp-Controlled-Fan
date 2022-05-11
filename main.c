//Nathan Nguyen
//Nelson Dane
/* EE260 Embedded Systems: SPECIAL project

Will be using temp sensor DHT11 from ardunio kit 
(with data pin connected to GPIO Port B pin 2)
	-Use power from the STM dev board for the temp sensor

Used Timer 8 for PWM and output channel is through PA5

*/

#include "stm32F446xx.h"
#include <stdio.h>  // For Printf

#define FAN_MAX 26667  												 // Global variable for max fan duty cycle
																							 //assuming 16 MHz system clock
void delayms(int n);                    			 //milliseconds
void delayMicroSeconds(int n);
void USART2_init(void);                 			 // Default GPIO PA2 
void USART2_wr_String(char* string);           // write string through USART
void DHT_WakeAndRead(float *output, int size); // To request data from the temperature sensor and wake it up from its automatic sleep state
void TIM3_init(void);  												 // Initialize TIM3 for determining long highs and short highs in DHT11 serial data to write down "0" or "1" in output array
void TIM8_init(void); 												 // Initialize TIM8 Channel 1 for PWM
void setFanSpeed(double n); 									 // Function to set the fan speed and output it to USART2, n is a percentage 0 to 100
void TIM2_init(void);                          // Used to implement a periodic interrupt that checks temperature sensor and adjust fan speed accordingly


int main(void){

	__disable_irq();
	
	RCC->AHB1ENR |= 2;   					//enable GPIOB clock
	GPIOB->MODER &= ~0x00000030; 	//reseting PB2, used by DHT11 for serial data
	
	TIM2_init();
	TIM3_init();
	TIM8_init();
	USART2_init();				
	
	NVIC_EnableIRQ(TIM2_IRQn);  	 //enable interrupt in NVIC
	
	delayms(1200);								 //wait 1.2 seconds in the beginning for the DHT11 sensor to get out of unstable state at every startup (according to data sheet)
	USART2_wr_String("------------INITIALIZATION------------\r\n");	
	
	__enable_irq(); 							 //(interrupt request service)
	
	while(1){  		
	}
}





void TIM2_IRQHandler(void) {
	
	float data[2] = {0};					 //index 0 is humidity and index 1 is temperature 
	#define hum data[0]
	#define temp data[1]	
	
	char printBuf[60];
		
	DHT_WakeAndRead(data,2);		
	sprintf((char*)printBuf, "Relative Humidity: %u%% | Temperature: %0.1fC \r\n", (int)data[0], data[1]);		
	USART2_wr_String(printBuf);

	
	
	if((hum > (float)60) || (temp > (float)28)){  
		setFanSpeed(75);
	}
	else if((hum > (float)47) || (temp > (float)27.1)){
		setFanSpeed(33);	
	}
	else{
		setFanSpeed(0);  
	}

	
	
	USART2_wr_String("\r\n");
	TIM2->SR=0;                    //reset interrupt flag
}



void TIM2_init(void){
		/* setup TIM2 */
    RCC->APB1ENR |= 1;              /* enable TIM2 clock */
    TIM2->PSC = 16000 - 1;          /* divided by 16000 */
    TIM2->ARR = 2000 - 1;           /* divided by 2000 for 0.5 Hz */
    TIM2->CR1 = 1;                  /* enable counter */
    TIM2->DIER |= 1;                /* enable UIE (update interrupt flag) */
}






//parameter is the return array output of the function
void DHT_WakeAndRead(float *output, int size){ 
		
	uint8_t wakeupDelay = 18;	  //according to datasheet
	uint8_t FAIL = 0;           //used if timeout happens somewhere         
	uint8_t mask = 128;   			//used to write "1" or "0" in output data from data signal timings
	uint8_t idx = 0;    				//index of array	
	uint8_t bits[5];            //raw data, there are 5 bytes														
	//NOTE:
  //the data is not explictly in the signal, the signal only tells us when to write down a "1" or a "0" depending on time of how long the high logic lasts for
	
	uint8_t timeout = 100;   		/* Max timeout is 100 microseconds */
	uint16_t PB2mask = 0x0004; 	//only show PB2 Input data register, hide the input data of all the other pins with & operation		
	uint16_t last;              //used to compare last timer value with current timer value for determining to write down a "1" or "0"



	/* EMPTY BUFFER */
	uint8_t i;
	for ( i = 0; i < 5; i++) bits[i] = 0;  //populate raw bits to be zero first
	
	
	
	
	
	/* REQUEST SAMPLE / WAKEUP */	
	GPIOB->MODER |=  0x00000010; 			// output mode PB2 pin
	GPIOB->ODR &= ~0x0004; 						// drive low
	delayms(wakeupDelay);							// wait for DHT to detect low 
	GPIOB->ODR |=  0x0004; 						// drive high 		
	delayMicroSeconds(40);  					// give time for the DHT 
    
	GPIOB->MODER &= ~0x00000030; 		  // input mode PB2 pin 	
	
	
	
	
	
	
	// GET ACKNOWLEDGE or TIMEOUT 
																							/* 
																							 *	watching drive LOW from the DHT
																							 */
	SysTick->LOAD = (16*timeout)-1;   					/* reload with number of clocks per (timeout value) microseconds */
  SysTick->VAL = 0;       										/* clear current value register */
  SysTick->CTRL = 0x5;    										/* Enable the timer, no interrupt, use system clock*/
	while ((GPIOB->IDR & PB2mask) == 0 ){      //wait/reading the low signal from DHT, shouldn't last too long though
		if ((SysTick->CTRL & 0x10000) != 0) {    //did COUNTFLAG set or go high?
			output[1] = FAIL; 													 //we waited too long
		}
	}
	SysTick->CTRL = 0;     										 /* Stop the timer (Enable = 0) */
	
																						/* 
																						 *	watching drive HIGH from the DHT
																						 */
	SysTick->LOAD = (16*timeout)-1;   				/* reload with number of clocks per (timeout value) microseconds */
  SysTick->VAL = 0;       									/* clear current value register */
  SysTick->CTRL = 0x5;    									/* Enable the timer, no interrupt, use system clock*/
	while ((GPIOB->IDR & PB2mask) != 0 ){      //wait/reading the high signal from DHT, shouldn't last too long though
		if ((SysTick->CTRL & 0x10000) != 0) {    //did COUNTFLAG set or go high?
			output[1] = FAIL; 													 //we waited too long
		}
	}
	SysTick->CTRL = 0;      									/* Stop the timer (Enable = 0) */	
																							/* ACK is complete
																							 * as the MCU sends a low high to wakeup and DHT sends a low high in response 
																							 * and that is the ACK/handshake
																							 *
																							 * These two whiles loops above me make sure that the low 
																							 * does not last too long and the high does not last for too long and cause a timeout flag
																							 */
	
	
  // READ THE OUTPUT - 40 BITS => 5 BYTES	     (Note: the DHT always sends a low first and then a high, this low-high pattern represents sending one bit)
	for (i=40; i!=0; i--){
																								/* 
																								*	watching drive LOW from the DHT
																								*/
		SysTick->LOAD = (16*timeout)-1; 						  /* reload with number of clocks per (timeout value) microseconds */
		SysTick->VAL = 0;     										  /* clear current value register */
		SysTick->CTRL = 0x5;    										/* Enable the timer, no interrupt, use system clock*/
		while ((GPIOB->IDR & PB2mask) == 0 ){      //wait/reading the low signal from DHT, shouldn't last too long though
			if ((SysTick->CTRL & 0x10000) != 0) {    //did COUNTFLAG set or go high?
				output[1] = FAIL; 													 //we waited too long
			}
		}
		SysTick->CTRL = 0;     										 /* Stop the timer (Enable = 0) */

		
		
																								//need to store last counter value here for determining "1" or "0"
		TIM3->CNT = 0;       												//clear timer counter  
		last = TIM3->CNT;	
		
		
		
																								/* 
																								 *	watching drive HIGH from the DHT
																								 */
		SysTick->LOAD = (16*timeout)-1;						   /* reload with number of clocks per (timeout value) microseconds */
		SysTick->VAL = 0;      											 /* clear current value register */
		SysTick->CTRL = 0x5;  										  /* Enable the timer, no interrupt, use system clock*/
		while ((GPIOB->IDR & PB2mask) != 0 ){      //wait/reading the high signal from DHT, shouldn't last too long though
			if ((SysTick->CTRL & 0x10000) != 0) {    //did COUNTFLAG set or go high?
				output[1] = FAIL; 													 //we waited too long
			}
		}
		SysTick->CTRL = 0;     										 /* Stop the timer (Enable = 0) */
		
		
		
		
		
		
		
		
		
		if(((TIM3->CNT)-last) > 40){ 						// long high time means output data 1-bit "1"
			bits[idx] |= mask;         						 // placing/forcing a "1" in the raw data array due to seeing a long high time			
		}   
		
    mask >>= 1;  														// short high time means output data 1-bit "0"
		// (this means 0b 0100 0000)
		// The assumption is that if we want to put a "0" in the raw data array, we just skip this bit because the whole array is already all zeros		
    
		
		if (mask == 0){   											// moving to next byte    
      mask = 128;   												// this means 0b 1000 0000
      idx++;        											  
    }				
	}	
	GPIOB->MODER |=  0x00000010; 								// output mode PB2 pin
	GPIOB->ODR |=  0x0004; 											// drive high, this tells the DHT to go back to sleep mode
	
	//humidity
	output[0] = bits[0];  											// bits[1] == 0 always, index 1 holds decimal values
	output[1] = bits[2] + (bits[3]*0.1); 				// index 3 holds decimal values
	//temperature
	
  // TEST CHECKSUM  
  if (bits[4] != (bits[0] + bits[1] + bits[2] + bits[3]))
  {
      output[1] = 420;                        //temperature is set to 420 to indicate checksum failure in UART terminal of computer
  }
}

void TIM3_init(void){
	/* setup TIM3 */                //this is for counting microseconds	
  RCC->APB1ENR |= 2;              /* enable TIM3 clock */
  TIM3->PSC = 16-1;       		   	/* divided by 16 to make a 1e6 Hz clock*/
  TIM3->ARR = 65000-1;           	/* This is the max number of microseconds it can count up to*/
	TIM3->CNT = 0;                 	//clear timer counter  
	TIM3->CR1 = 1;                  /* enable counter */	
}

void TIM8_init(void) {
    
    RCC->AHB1ENR |= 1;              // Enable GPIOA clock
    
    GPIOA->AFR[0] |= 0x00300000;    // PA5 pin for TIM8
    GPIOA->MODER &= ~0x00000C00;
    GPIOA->MODER |=  0x00000800;
    
    /* setup TIM8 */
    RCC->APB2ENR |= 2;              // Enable TIM8 clock
    TIM8->PSC = 10 - 1;             // divided by 10
    TIM8->ARR = 26667 - 1;          // divided by 26667
    TIM8->CNT = 0;
    TIM8->CCMR1 = 0x0068;           // PWM mode
    TIM8->CCER = 4;                 // enable PWM Ch1N
    TIM8->CCR1 = 0;               	// initial pulse width
    TIM8->BDTR |= 0x8000;           // enable output
    TIM8->CR1 = 1;                  // enable timer
}

// Function to set the fan speed and output it to USART2, n is a percentage 0 to 100
void setFanSpeed(double n) {
    char output[50]; 																							// Create character array to hold output string
    TIM8->CCR1 = (double)(FAN_MAX*n)/(double)100; 								// Set TIM8 Channel 1 equal to the max fan speed times n as a percentage
    sprintf((char*)output,"Fan speed: %u%% \r\n",(int)n); 			  // Create output message character array
    USART2_wr_String(output); 																		// Print out fan speed as a percentage
}

void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2 for USART2 TX */
    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x0030;   //reset just PA2
    GPIOA->MODER  |=  0x0020;   /* enable alternate function for PA2 */

    USART2->BRR = 0x008B;       /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x0008;       /* enable Tx, 8-bit data */ 
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

void USART2_wr_String(char* string)
{
	uint8_t ch;
	for(ch=0;string[ch] !=0;ch++)
	{
	  while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (string[ch] & 0xFF);
	}
}

//assuming 16MHz default sysclk and func parameter is in milliseconds
//designed to use systick instead of loops
void delayms(int n){ 
	int i;	
	/* Configure SysTick */
  SysTick->LOAD = 16000-1;  								 /* reload with number of clocks per millisecond */
  SysTick->VAL = 0;       									 /* clear current value register */
  SysTick->CTRL = 0x5;    									 /* Enable the timer, no interrupt, use system clock*/

  for(i = 0; i < n; i++) {                 	 //number of milliseconds to count
    while((SysTick->CTRL & 0x10000) == 0)    /* wait until the COUNTFLAG is set, this only counts up to 1 millisecond */
    { }
  }
  SysTick->CTRL = 0;      									 /* Stop the timer (Enable = 0) */
	
}

void delayMicroSeconds(int n){ 
	int i;	
	/* Configure SysTick */
  SysTick->LOAD = 16-1;  										 /* reload with number of clocks per microsecond */
  SysTick->VAL = 0;       									 /* clear current value register */
  SysTick->CTRL = 0x5;    									 /* Enable the timer, no interrupt, use system clock*/

  for(i = 0; i < n; i++) {                 	 //number of microseconds to count
    while((SysTick->CTRL & 0x10000) == 0)    /* wait until the COUNTFLAG is set, this only counts up to 1 microsecond */
    { }
  }
  SysTick->CTRL = 0;      									 /* Stop the timer (Enable = 0) */	
}
