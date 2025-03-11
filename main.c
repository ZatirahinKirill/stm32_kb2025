#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>

//задержка в миллисекундах
void delay_us(uint32_t us){
    __asm volatile (
        //задержка в миллисекундах (ASM)
        //push{r0}
        //mov R0, %val     //val = (9 * us) for 72Mhz
        //_loop:           //approx. 8ticks/iteration
        //    cmp R0, #0    //1
        //    beq_exit      //1 or 1+P (when condition is True)
        //    sub R0,R0,#1  //1
        //    nop           //1 allignment
        //    b _loop       //1+P (pipeline refill) ~4 cycle
        //_exit:
        //    pop{r0}
        
        "push {r0}\r\n"
        "mov R0, %0\r\n"
        "_loop:\r\n"
            "cmp R0, #0\r\n"
            "beq _exit\r\n"
            "sub R0, R0, #1\r\n"
            "nop\r\n" 
            "b _loop\r\n"
        "_exit:\r\n"
            "pop {r0}\r\n"
        :: "r"(9*us) // for 72 Mhz
    );
}

/* Interrupt handler */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        if(GPIOC->ODR & GPIO_ODR_ODR13){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else {
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }
    //Clear Interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;
    }
}

int main(void) {
    // int i = 0;
    // int mask = 8; // 8 = 0b10000 = 0x8 = (1 << 4)
    // i = i | mask; // i |= mask;
    
    /* IO PORTS Configuration */
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // 0b10000=0x10
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); // GPIOC->CRH[23:20]=0000
    GPIOC->CRH |= GPIO_CRH_MODE13_0; // GPIOC->CRH[23:20]=0001
    
    //Port PB0 as Input
    //Включаем тактовый сигнал порт B
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    //Настраиваем режим input
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // GPIOB->CRL[3:0]=0000
    GPIOB->CRL |= GPIO_CRL_CNF0_1; // GPIOB->CRL[3:0]=1000
    //Настраиваем режим pull-up resister
    GPIOB->ODR |= GPIO_ODR_ODR0; // PB0 Internal pull-up resister
    
    //таймер
    /* TIM2 Configuration */
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    TIM2->PSC = 4000;
    TIM2->ARR = 18000;
    TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer
    while (1) {
        __asm volatile ("nop");
    }
    
   /*
    while (1){
        if(GPIOB->IDR & GPIO_IDR_IDR0){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else{
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }
    }
    */
    //просто мигает
    /*
    while(1){
        GPIOC->ODR &= ~GPIO_ODR_ODR13;
        delay_us(1000000);
        GPIOC->ODR |= GPIO_ODR_ODR13;
        delay_us(1000000);
    }
    */

return 0;
}
