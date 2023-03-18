/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file       	: main.c
  * @brief      	: Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
     void turn_on_LED(char);
     void turn_off_LED(char);
     void configure_uart(void);
     void initialize_uart(void);
     void transmitChar(char);
     void transmitString(char*);
     void transmitValue(uint16_t);
     void calibrate_ADC(void);
     void calibrate_ADC_manual(void);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

   // Define LED threshold values (70 top)
     uint16_t RED_BOUND = 50;
     uint16_t ORANGE_BOUND = 100;
     uint16_t GREEN_BOUND = 175;
     uint16_t BLUE_BOUND = 250;
		 
	 // Sin wave values
		const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
		232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
    
    /***** PART 2 *****/
		
		/*** Peripheral Config ***/
		// Initialize PA4 to DAC_OUT1 Alt mode
		RCC->APB1ENR |= RCC_APB1ENR_DACEN;
		GPIOA->MODER &= ~((1 << 8) | (1 << 9));	// Clear
		GPIOA->MODER |= ((1 << 8) | (1 << 9));	// MODER4 to 11
		
		/*** DAC Config ***/
		// Turn on software trigger for DAC1
		DAC1->SWTRIGR &= ~(0x1);
		DAC1->SWTRIGR |= 0x1;
		
		// Enable DAC channel
		DAC1->CR &= ~(0x1); // Clear
		DAC1->CR |= 0x1;		// EN1=1
		
		
		
		/***** PART 1 *****/
    /*** Peripheral Config ***/
    // Initialize LEDs to output mode
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0xFF000); // Clear
    GPIOC->MODER |= 0x55000;   	 // Set to general purpose output mode
   	 
    // Configure PC0 to analog mode
    GPIOC->MODER &= ~(0x3);   		 // Clear
    GPIOC->MODER |= 0x3;
    
    // Enable ADC1 in RCC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    
		
		
    /*** DEBUGGING ***/
    configure_uart();
    initialize_uart();
    
		
		
    /*** Configure ADC ***/
    // Turn to 8-bit resolution
    ADC1->CFGR1 &= ~(1 << 4);
    ADC1->CFGR1 |= (1 << 4);
    
    // Turn on continuous conversion mode
    ADC1->CFGR1 &= ~(1 << 13);
    ADC1->CFGR1 |= (1 << 13);
    
    // Turn off hardware triggers
    ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));
    
    // Enable pin 0 for ADC conversion (todo: this might be physical pin 8 val?)
    // I also tried (1 << 8) and that did not work
    ADC1->CHSELR &= ~(1 << 10);
    ADC1->CHSELR |= (1 << 10);
    
		
    
    /*** Calibrate ADC ***/
    calibrate_ADC_manual();
    
    // Ensure ADSTP, ADSTART, ADDIS are = 0
    ADC1->CR &= ~(1 << 4); //STP
    ADC1->CR &= ~(1 << 2); //START
    ADC1->CR &= ~(1 << 1); //DIS
    
    // Enable ADC
    ADC1->CR &= ~(0x1);
    ADC1->CR |= 0x1;
    
    // Start conversion
    ADC1->CR |= (1 << 2);
		
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// Set up reading loop
  uint16_t data = 0;
	
	// Set up writing loop
	uint8_t i = 0;
  while (1)
  {
		
		 /***** PART 1 *****/
   	 // Read ADC data reg and turn on/off LEDs accordingly
   	 data = ADC1->DR;
   	 transmitValue(data);
   	 
   	 // Turn on red if above 0
   	 if (data > RED_BOUND) {
   		 turn_on_LED('r');
   	 } else {
   		 turn_off_LED('r');
   	 }
   	 
   	 // Turn on orange if above 1
   	 if (data > ORANGE_BOUND) {
   		 turn_on_LED('o');
   	 } else {
   		 turn_off_LED('o');
   	 }
   	 
   	 // Turn on green if above 2
   	 if (data > GREEN_BOUND) {
   		 turn_on_LED('g');
   	 } else {
   		 turn_off_LED('g');
   	 }
   	 
   	 // Turn on blue if at or greater than 3
   	 if (data >= BLUE_BOUND) {
   		 turn_on_LED('b');
   	 } else {
   		 turn_off_LED('b');
   	 }
   	 
   	 
		
		 /***** PART 2 *****/
		 // Write next wave value into register
		 DAC1->DHR8RD = sine_table[i];
		 i++;
		 i = i % 32;
		 HAL_Delay(1);
		 
  }
  /* USER CODE END 3 */
}

void configure_uart(void) {
    // Set PC10 and PC11 to alt fx mode
    GPIOC->MODER &= ~(0xF00000);    // Clear
    GPIOC->MODER |= 0xA00000;   		 // 1010
    
    // Set PC10 to AF1 for USART3_TX/RX
    GPIOC->AFR[1] &= ~(0xFF00);   	 // Clear
    GPIOC->AFR[1] |= 0x1100;   		 // Set 11 and 10 to 0001
}

/* Pulled straight from the Appendix of the Peripheral Manual */
void calibrate_ADC_manual(void) {
   	//  transmitString("Calibrating ADC with manual version...");
    
    // Ensure ADEN = 0
    if ((ADC1->CR & ADC_CR_ADEN) != 0) {
   	 // Clear ADEN
   	 ADC1->CR |= ADC_CR_ADDIS;
    }
    while ((ADC1->CR & ADC_CR_ADEN) != 0) {
   	 HAL_Delay(500);
    }
    // Clear DMAEN
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
    
    // Launch calibration
    ADC1->CR |= ADC_CR_ADCAL;
    
    // Wait until ADCAL = 0
    while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
   	 HAL_Delay(500);
    }
}

void initialize_uart(void) {
    // Enable system clock for USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    // Set Baud rate
    uint64_t t_baud = 115200;
    uint64_t f_clk = HAL_RCC_GetHCLKFreq();
    uint64_t brr = f_clk/t_baud;
    USART3->BRR &= ~(0xFFFFFFFF);   	 // Clear
    USART3->BRR |= brr;

    // Enable tx
    USART3->CR1 |= (1 << 3);
    
    // Enable rx
    USART3->CR1 |= (1 << 2);
    
    // Enable rx not-empty interrupt
    USART3->CR1 |= (1 << 5);
    
    // Enable USART
    USART3->CR1 |= (1 << 0);
    
    // Enable USART3 interrupt on NVIC
    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn, 1);
}

// Transmits argument character over USART3
void transmitChar(char c) {
    
    // Wait until our tx reg is ready
    volatile int wait = 1;
    while (wait) {
   	 // Check TC bit
   	 if (USART3->ISR & 0x80) {
   		 wait = 0;
   	 }
    }
    // Write to tx reg
    USART3->TDR = c;
}

// Transmits argument string over USART3
void transmitString(char* s) {
    int idx = 0;
    char c = s[idx];
    while (c != 0) {
   	 transmitChar(c);
   	 idx++;
   	 c = s[idx];
    }
    transmitChar('\r');
    transmitChar('\n');
}

void transmitValue(uint16_t val) {
   	 // Wait until our tx reg is ready
    volatile int wait = 1;
    while (wait) {
   	 // Check TC bit
   	 if (USART3->ISR & 0x80) {
   		 wait = 0;
   	 }
    }
    // Write to tx reg
    USART3->TDR = val;
}

void turn_off_LED(char color) {
   	 switch (color) {
   		 case 'r':
   			 GPIOC->ODR &= ~(0x0040);
   			 break;
   		 case 'o':
   			 GPIOC->ODR &= ~(0x100);
   			 break;
   		 case 'g':
   			 GPIOC->ODR &= ~(0x200);
   			 break;
   		 case 'b':
   			 GPIOC->ODR &= ~(0x80);;
   			 break;
   	 }
}

void turn_on_LED(char color) {
   	 switch (color) {
   		 case 'r':
   			 GPIOC->ODR |= 0x0040;
   			 break;
   		 case 'o':
   			 GPIOC->ODR |= 0x100;
   			 break;
   		 case 'g':
   			 GPIOC->ODR |= 0x200;
   			 break;
   		 case 'b':
   			 GPIOC->ODR |= 0x80;
   			 break;
   	 }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          	|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
	Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *     	where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
 	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


