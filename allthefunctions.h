#include <stm32f4xx.h>
void gpio_A_config(void);
void gpio_A_config(){
	RCC->AHB1ENR |= 0x1;
	GPIOA->MODER |= 0x00000000; // 11 analog 10 alternate 01 output 00 input
	GPIOA->ODR |= 0x00000000; 
}
void gpio_B_config(void);
void gpio_B_config(){
	RCC->AHB1ENR |= 0x2;
	GPIOB->MODER |= 0x50050415; // 11 analog 10 alternate 01 output 00 input
	GPIOB->ODR |= 0x00000000; 
}
void gpio_C_config(void);
void gpio_C_config(){
	RCC->AHB1ENR |= 0x4;
	GPIOC->MODER |= 0x00000000; // 11 analog 10 alternate 01 output 00 input
	GPIOC->ODR |= 0x00000000; 
}

void systick_delay(int,int);
void systick_delay(int load_value,int delay){
	//2^24 values can be stored close to 16 million so load can not be greater than that 
	//delay should be a multiple of load_value
	SysTick->LOAD = load_value; // 1s delay at 16MHz
  SysTick->VAL = 0; // Reset counter
	SysTick->CTRL = 0x5;
	for(int i=0;i<delay;i++){
		if (SysTick->CTRL & 0x10000) { // Wait for COUNTFLAG (1s has passed)
            i++;
		}
	}
}
void Timer2_Init(void);
void Timer2_Init(void) {
    RCC->APB1ENR |= (1 << 0);  // Enable TIM2 clock
    TIM2->PSC =168000 - 1; 
}
void delay(int,int);
void delay(int ms,int higher){
	// delay is in millisecond it is the active delay the register is 16 bits each for prescale and reload register
	// delay should be ms*higher
	
	TIM2->ARR = ms - 1;
	TIM2->CNT = 0;
	TIM2->CR1 |= (1 << 0);
	for(int i=0;i<higher;i++){	
		while(!(TIM2->SR & TIM_SR_UIF)){} 
				TIM2->SR&= ~TIM_SR_UIF;
		}	
		TIM2->CR1 &= ~(1 << 0); 
}
void Tim2_start(void);
void Tim2_start(){
    TIM2->CNT = 0;                // Reset counter
    TIM2->ARR = 0xFFFFFFFF;       // Max ARR (32-bit), timer runs freely
    TIM2->CR1 |= TIM_CR1_CEN;     // Start timer
}
uint32_t Tim2_value(void);
uint32_t Tim2_value(void) {
    //TIM2->CR1 &= ~TIM_CR1_CEN;    // Stop timer
    return TIM2->CNT;             // Read elapsed time in milliseconds
}
void ADC1_Init(void);
void ADC1_Init(void) {
    RCC->APB2ENR |= (1 << 8);      // Enable ADC1 clock
    RCC->AHB1ENR |= (1 << 0);      // Enable GPIOA clock
    ADC1->CR2 |= (1 << 1);         // Continuous conversion mode
    ADC1->CR2 |= (1 << 10);        // EOC after each conversion
    ADC1->CR2 |= (1 << 0);         // ADON - enable ADC
    ADC1->SQR3 = 0;                // Channel 0 selected (PA0)
    GPIOA->MODER |= (3 << 0);      // PA0 in analog mode
}

uint16_t Read_ADC1(void);
uint16_t Read_ADC1(void) {
    ADC1->SR = 0;                  // Clear status register
    ADC1->CR2 |= (1U << 30);       // Start conversion (SWSTART)
    while (!(ADC1->SR & (1 << 1))); // Wait for EOC
    return ADC1->DR;
} 

void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char *str);
char UART1_ReceiveChar(void);
void UART1_ReceiveLine(char *buffer, int max_len);

void UART1_Init(void) {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock

    // Configure PA9 (TX) and PA10 (RX) as Alternate Function (AF7)
    GPIOA->MODER &= ~((3 << (2 * 9)) | (3 << (2 * 10)));     // Clear bits
    GPIOA->MODER |= (2 << (2 * 9)) | (2 << (2 * 10));        // Set AF mode

    GPIOA->AFR[1] &= ~((0xF << (4 * 1)) | (0xF << (4 * 2))); // Clear AF
    GPIOA->AFR[1] |= (7 << (4 * 1)) | (7 << (4 * 2));        // AF7 (USART1)

    // USART1 configuration
    USART1->BRR = 0x0683;  // Baud rate 9600 (assuming 16 MHz clock)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable TX, RX
    USART1->CR1 |= USART_CR1_UE;                 // Enable USART
}

void UART1_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));  // Wait until TXE (Transmit empty)
    USART1->DR = c;
}

void UART1_SendString(const char *str) {
    while (*str) {
        UART1_SendChar(*str++);
    }
}

char UART1_ReceiveChar(void) {
    while (!(USART1->SR & USART_SR_RXNE));  // Wait until RXNE (Data ready)
    return USART1->DR;
}

void UART1_ReceiveLine(char *buffer, int max_len) {
    int idx = 0;
    char c;
    while (idx < max_len - 1) {
        c = UART1_ReceiveChar();
        UART1_SendChar(c);  // Echo
        if (c == '\r' || c == '\n') {
            break;
        }
        buffer[idx++] = c;
    }
    buffer[idx] = '\0';
    UART1_SendString("\r\n");  // Move to new line
}

void UART3_Init(void);
void UART3_SendChar(char c);
void UART3_SendString(char *str);
char UART3_ReceiveChar(void);
 
void UART3_Init(void) {
    // Enable clocks for GPIOD and USART3
    RCC->APB1ENR |= (1 << 18); // USART3 clock enable
    RCC->AHB1ENR |= (1 << 3);  // GPIOD clock enable
 
    // Configure PD8 (TX) and PD9 (RX) as Alternate Function (AF7)
    GPIOD->MODER |= (1 << 17) | (1 << 19); // Set mode to AF
    GPIOD->AFR[1] |= 0x00000077; // Set alternate function to AF7 (USART3)
 
    // Configure USART3
    USART3->BRR = 0x0683; // Baud rate for 9600 bps (assuming 16MHz clock)
    USART3->CR1 |= (1 << 13); // Enable USART3
    USART3->CR1 |= (1 << 2) | (1 << 3); // Enable RX and TX
}
 
void UART3_SendChar(char c) {
    while (!(USART3->SR & (1 << 7))); // Wait until TXE is set (empty)
    USART3->DR = c;
}
 
void UART3_SendString(char *str) {
    while (*str) {
        UART3_SendChar(*str++);
    }
}
 
char UART3_ReceiveChar(void) {
    while (!(USART3->SR & (1 << 5))); // Wait until RXNE (Read Data Register Not Empty) is set
    return USART3->DR;
}
void OnButtonPress(void);
void OnButtonPress(void) {
    //empty function to write the action you need to perform
}

void EXTI15_10_IRQHandler(void);
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << 13)) {    // Check if EXTI13 triggered
        OnButtonPress();
        EXTI->PR = (1 << 13);      // Clear pending interrupt
    }
}

void EXTI_Init(void);
void EXTI_Init(void) {
    RCC->APB2ENR |= (1 << 14);     // Enable SYSCFG clock

    SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Clear EXTI13 config
    SYSCFG->EXTICR[3] |= (2 << 4);     // Set EXTI13 to port C

    EXTI->IMR |= (1 << 13);        // Unmask EXTI13
    EXTI->RTSR |= (1 << 13);       // Rising edge trigger

    NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void Timer2_Interrupt_Init(void);
void TIM2_IRQHandler(void);
void OnTimerInterrupt(void);
void Timer2_Interrupt_Init(void) {
    // 1. Enable clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 2. Set prescaler and ARR for 1ms interrupt
    TIM2->PSC = 16000 - 1;    // Prescaler (16 MHz / 16000 = 1 kHz)
    TIM2->ARR = 1000 - 1;        // Auto-reload value for 1 s

    // 3. Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // 4. Enable TIM2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // 5. Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

// ISR (Interrupt Service Routine)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {   // Check update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;   // Clear the flag
        OnTimerInterrupt();        // Call your custom function
    }
}

// Custom function to run on every timer interrupt
void OnTimerInterrupt(void) {
    // Empty to modulate
}
void Timer5_Init(void);
void Timer5_Reset(void);
uint32_t Timer5_Elapsed(void);
void Timer5_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;  // Enable TIM5 clock
    TIM5->PSC = 0x1481F;               // Prescaler for 1ms tick (assuming 168MHz system clock)
    TIM5->ARR = 0xFFFF;              // Max auto-reload (free running)
    TIM5->CNT = 0;                       // Reset counter
    TIM5->CR1 |= TIM_CR1_CEN;           // Start timer
}

void Timer5_Reset(void) {
    TIM5->CNT = 0;  // Manually reset the counter
}

uint32_t Timer5_Elapsed(void) {
    return TIM5->CNT; // Return elapsed time in ms (since prescaler gives 1 ms ticks)
}
