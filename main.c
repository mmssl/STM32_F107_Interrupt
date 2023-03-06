
#include "stm32f10x.h"

void initClockHSI (void);
void initGPIO (void);
void configInterrupts(void);
void delay (uint32_t time);
void EXTI4_IRQHandler(void);


uint8_t count = 0;
int flag = 0;



int main()
{
	initClockHSI(); // set system clock HSI
	initGPIO();
	configInterrupts();
	char counter = 0;
	GPIOD->ODR &= ~(0xF);
	
	while(1)
	{

			if (flag)
			{
				delay(99999);  // avoid debouncing
				//GPIOD->ODR ^= (1<<0);
				GPIOD->ODR++;
				count++;
				flag = 0;
			}			

	}

return 0;
}



void EXTI4_IRQHandler(void)
{
	/****>>>>>>> To be Completed <<<<<<<<*****
	1. Check the Pin, which trigerred the Interrupt
	2. Clear the Interrupt Pending Bit
	3. Update flag variable
	********************/
	
	if (EXTI->PR & (1<<4))
	{
	
	EXTI->PR |= (1<<4);
	flag = 1;
	}
		
}


void configInterrupts (void)
{
	/****>>>>>>> To be Completed  <<<<<<<<*****
	1. Enable the AFIO CLOCK bit in RCC register 
	2. Configure the EXTI configuration Regiter in the AFIO
	3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt
	********************/
	
	
	/*Enable the AFIO CLOCK bit in RCC register*/
	RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	 /* Configure the EXTI configuration Regiter in the AFIO */
	
		AFIO->EXTICR[1] &= ~(0x3); //~AFIO_EXTICR2_EXTI4_PD);

	  AFIO->EXTICR[1] |= (0x3);
	
	
	/* Disable the EXTI Mask using Interrupt Mask Register (IMR) */
	
	EXTI->IMR |= (1<<4);
	
	/* Configure the Rising Edge / Falling Edge Trigger */
	
	EXTI->RTSR |= (1<<4);
	
	EXTI->FTSR &= ~(1<<4);
	
	/* Set the Interrupt Priority */
	
	NVIC_SetPriority(EXTI4_IRQn, 1);
	
	/* Enable the interrupt */
	NVIC_EnableIRQ(EXTI4_IRQn);
	
}

void initGPIO (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // Enable port D clock
	
	GPIOD->CRL &= ~(0xffff); //~(GPIO_CRL_MODE0) & ~(GPIO_CRL_MODE1) & ~(GPIO_CRL_MODE2) & ~(GPIO_CRL_MODE3);
	GPIOD->CRL |= GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE3;  //Port D pin 0,1 set as output
	//GPIOD->CRL &= ~(GPIO_CRL_CNF0) & ~(GPIO_CRL_CNF1) & ~(GPIO_CRL_CNF2) & ~(GPIO_CRL_CNF3);
}


void initClockHSI (void)
{
	RCC->CR|= RCC_CR_HSION; // enable internal HSI(RC)
	
	while((RCC->CR & RCC_CR_HSIRDY)!=RCC_CR_HSIRDY);  // wait HSI to be ready
	
	RCC->CFGR &= ~(RCC_CFGR_SW); //clear SW bits
	RCC->CFGR |= RCC_CFGR_SW_HSI; //set HSI as system clock
	
	while( (RCC-> CFGR & RCC_CFGR_SWS_HSI)!= RCC_CFGR_SWS_HSI); // wait HSI to be system clock
}



void delay (uint32_t time)
{
	while(time--);
}

