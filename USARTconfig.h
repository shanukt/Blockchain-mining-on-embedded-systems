#include "stm32f4xx.h"

void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char *str);
char UART1_ReceiveChar(void);
void UART1_ReceiveLine(char *buffer, size_t max_len);

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

void UART1_ReceiveLine(char *buffer, size_t max_len) {
    size_t idx = 0;
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
