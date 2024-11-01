void init_tim6(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    //Set prescaler to 2 - 1
    TIM6->PSC = 2 - 1;
    //Calculate ARR for 30 Hz interrupt rate
    TIM6->ARR = (24000000 / 30) - 1;
    //Enable update interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 1 << 17;
    //Enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
}

void init_tim6(void) { 
    //Enable RCC clock for TIM6 
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; 
    //Set prescaler to 2 - 1 
    TIM6->PSC = 2 - 1; 
    //Calculate ARR for 30 Hz interrupt rate 
    TIM6->ARR = (24000000 / 30) - 1; 
    //Enable update interrupt 
    TIM6->DIER |= TIM_DIER_UIE; 
    //Unmask the interrupt in the 
    NVIC NVIC->ISER[0] = 1 << 17; 
    //Enable counter 
    TIM6->CR1 |= TIM_CR1_CEN; TIM6->CR2 |= TIM_CR2_MMS_1; 
}
