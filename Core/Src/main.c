/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * @authors        : Rodrigo De Lama Fernández @rodrigodelama
  *                   Manuel Morales Niño @ikaoseu
  *                   Jaime Mato Rodriguez @Pekeniojimi
  *
  * @repo           : https://github.com/rodrigodelama/Micros_1.2
  * 
  * former repo with
  * commit history  : https://github.com/rodrigodelama/LAB_1
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l152c_discovery.h"
#include "stm32l152c_discovery_glass_lcd.h"
#include "SDM_Utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//GLOBAL VARS & DEFINITIONS
unsigned int prev_game = 0;
unsigned int game = 1; //GAME1 is the starting point
unsigned int winner = 0; //Initialised to 0, if it never changes, it will generate an error
unsigned int playing = 0; //for game 1 - Maintains the button interrupts disabled unless we are awaiting
unsigned int btn_pressed = 0; //for game2 - To exit countdown loop

unsigned int time_3 = 0;
unsigned int time_4ch1 = 0;
unsigned int time_4ch2 = 0;
unsigned int time_2ch1 = 0; //TODO: turn on when TIM2 is finished

unsigned short delta;
unsigned int randn;
unsigned int low_limit_randn = 500;
unsigned int high_limit_randn;

unsigned int countdown;
unsigned int potentiometer_value;
unsigned int adc_value;
unsigned int x; // 1/3 point
unsigned int y; // 2/3 point

unsigned int timeout_time;
unsigned int timer_count_limit;

unsigned int octave3[7] = {130.812783, 146.832384, 164.813778, 174.614116, 195.997718, 220.000000, 246.941651};
unsigned int octave4[7] = {261.625565, 293.664768, 329.627557, 349.228231, 391.995436, 440.000000, 493.883301};

uint8_t text[6]; //ASCII character array to output in the Discovery LCD

short duty_cycle = 1; //TODO: for TIM2 PWM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//INTERRUPTS
void EXTI0_IRQHandler(void)
{
  // USER BUTTON is pressed, a rising edge is detected in PA0
  if ((EXTI->PR &BIT_0) != 0) // Is EXTI0 flag on? //0000000000000001 in the Pending Register of ISER[0]
  {
    game++; //If at GAME 1, proceed to GAME 2
    if (game > 2) game = 1; //Reset to 1 after requesting change from GAME 2
  }
  EXTI->PR |= (1 << 6); // Clear EXTI0 flag (writes a 1 in PR0 pos)
  //Clear flag must be out of condition so it never hangs if condition is not met
}

// NVIC->ISER[0] pin 23 for EXTIs 9 to 5
void EXTI9_5_IRQHandler(void) //ISR for EXTI7 & EXTI6
{
  if (((EXTI->PR &BIT_7) || (EXTI->PR &BIT_6)) != 0) //most general aproach --> (EXTI->PR != 0)
  {
    // BUTTON 1 is pressed, a rising edge is detected in PB7
    if (EXTI->PR &(1 << 7)) // 00000000010000000 in pending register of ISER[0]
    {
      if ((game == 1) && (playing == 1))
      {
        GPIOA->BSRR = (1 << 12) << 16; //Turn off LED
        playing = 0;
        winner = 1;
      }
      else if (game == 2)
      {
        //This code is meant to end the while loop, we will determine the winner by checking the time deltas afterwards
        btn_pressed = 1;
      }
      //else nothing
    }
    // BUTTON 2 is pressed, a rising edge is detected in PB6
    if (EXTI->PR &(1 << 6)) // 00000000001000000 in pending register
    {
      if ((game == 1) && (playing == 1))
      {
        GPIOA->BSRR = (1 << 12) << 16;
        playing = 0;
        winner = 2;
      }
      else if (game == 2)
      {
        btn_pressed = 2;
      }
    }
  }
  //Always clear flags to avoid hanging
  EXTI->PR |= (1 << 7); // Clear the EXTI7 flag (writes a 1 in PR7)
  EXTI->PR |= (1 << 6); // Clear the EXTI6 flag
}

//TIMERS
//TIC timer 4 CH1 and CH2
void TIM4_IRQHandler(void)
{
  //CH1 for PB6 - Player 2
  if((TIM4->SR &BIT_1) != 0) //0x2
  {
    time_4ch1 = TIM4->CCR1; //The time loaded to our var shall be the count when the button was clicked
    if(time_4ch1 < 0) time_4ch1 += 0x0FFFF; //FIXME: avoids impossible negative times ?? - grabbed from ENG_08_Solutions_Timers_v4.pdf, exercise 6
  }
  //CH2 for PB7 - Player 1
  if((TIM4->SR &BIT_2) != 0) //0x4
  {
    time_4ch2 = TIM4->CCR2;
    if(time_4ch2 < 0) time_4ch2 += 0x0FFFF;
  }
  TIM4->SR = 0x0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  GPIO_InitTypeDef GPIO_InitStruct; //For corrrect USART3 implementation with pins PC10 and PC11
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
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //DECLARATION OF PERIPHERALS WE WILL USE

  //LCD Setup
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_BarLevelConfig(0);
  BSP_LCD_GLASS_Clear();
  //For corrrect USART3 implementation with pins PC10 and PC11
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //Pins + their EXTIs - I/O
  /* PB0 ---------------------------------------------------------------------*/
  //PA0 (USER BUTTON) - digital input (00)
  GPIOA->MODER &= ~(1 << (0*2 + 1));
  GPIOA->MODER &= ~(1 << (0*2));
  //EXTI0
  SYSCFG->EXTICR[0] = 0000; // Linking EXTI0 to GPIOA (PA0 = USER BUTTON) - all zeros mean GPIOA
  EXTI->RTSR |= BIT_0; // Enables rising edge in EXTI0
  EXTI->FTSR &= ~(BIT_0); // Disables falling edge in EXTI0
  EXTI->IMR |= BIT_0; // Enables the Interrupt (i.e. the event) (IMR = Interrupt Mask Register)
  NVIC->ISER[0] |= (1 << 6); //Enables EXTI0 in the NVICs ISER[0]s position 6

  //USING I/O PINS 7 & 6 FOR PLAYER BUTTONS 1 & 2 RESPECTIVELY
  /* PB7 ---------------------------------------------------------------------*/
  //PB7 (BUTTON 1) - digital input (00) - w/ AF TIM4_CH2
  //Set PB7 to 10 for Alternate Functions (AFs)
  GPIOB->MODER |= (1 << (7*2 + 1));
  GPIOB->MODER &= ~(1 << (7*2));
  //AF for TIM4_CH2 (AF2)
  GPIOB->AFR[0] |= (0x02 << (7*4)); // Writes 0010 in AFRL7 associating TIM4 to PB7
  //WE NEED INTERNAL RESISTORS - pull-up OR pull-down ?
  //We chose pull-up: a constant 1 unless we press, then its shorted to GND
  //Set up with pull-up resistor (01)
  GPIOB->PUPDR &= ~(1 << (7*2 + 1));
  GPIOB->PUPDR |= (1 << (7*2));
  //EXTI7
  SYSCFG->EXTICR[1] |= BIT_12; // Linking EXTI7 to GPIOB (PB7 = BUTTON 1)
                    // Sets a 1 in bit 12 see page 145 of the manual
  EXTI->RTSR &= ~(BIT_7); // Disables rising edge in EXTI7
  EXTI->FTSR |= BIT_7; // Enables falling edge in EXTI7
  EXTI->IMR |= BIT_7; // Enables the interrupt after setup
  NVIC->ISER[0] |= (1 << 23); // EXTI7 has position 23 in ISER[0]

  /* PB6 ---------------------------------------------------------------------*/
  //PB6 (BUTTON 2) - digital input (00) - w/ AF TIM4_CH1
  //Set PB6 to 10 for AFs
  GPIOB->MODER |= (1 << (6*2 + 1));
  GPIOB->MODER &= ~(1 << (6*2));
  //AF for TIM4_CH1 (AF2)
  GPIOB->AFR[0] |= (0x02 << (6*4)); // Writes 0010 in AFRL6 associating TIM4 to PB6
  //Set up with pull-up resistor (01)
  GPIOB->PUPDR &= ~(1 << (6*2 + 1));
  GPIOB->PUPDR |= (1 << (6*2));
  //EXTI6
  SYSCFG->EXTICR[1] |= BIT_8; // Linking EXTI6 to GPIOB (PB6 = BUTTON 2)
                    // Sets a 1 in bit 8 see page 145 of the manual
  EXTI->RTSR &= ~(BIT_6); // Disables rising edge in EXTI6
  EXTI->FTSR |= BIT_6; // Enables falling edge in EXTI6
  EXTI->IMR |= BIT_6; // Enables the interrupt
  NVIC->ISER[0] |= (1 << 23); // EXTI6 & 7 have position 23 in the NVIC, since

  //TIMERS
  /* TIM3 --------------------------------------------------------------------*/
  //No pin assignment, we just plainly use it for the TOC
  //SET-UP for TIM3 - TOC, for random LED off and TBD
  TIM3->CR1 = 0x0000;   // ON IN CODE BELOW
  TIM3->CR2 = 0x0000;   // Always set to 0
  TIM3->SMCR = 0x0000;  // Always set to 0
  TIM3->PSC = 31999;    //Means fclk/(PSC+1)
  TIM3->CNT = 0;        //Initiallized to 0
  TIM3->ARR = 0xFFFF;   //USED IN PWN so set to max
  TIM3->DIER = 0x0000;  
  TIM3->CCMR1 = 0x0000; //CCyS = 00 (TOC)
                        //OCyM = 000 (no external output)
                        //OCyPE = 0 (no preload)
  TIM3->CCER = 0x0000;  //CCyP = 0 (always in TOC)
                        //CCyE = 0 (external output disabled)
  /* TIM4 --------------------------------------------------------------------*/
  //Assigned to PB7 and PB7
  //SET-UP for TIMs 4, CH1 & CH2 - TICs
  TIM4->CR1 = 0x0000;   //Set to 0 for Counter OFF initially
                        //It will be turned ON in code below
                        //ARPE off because NOT PWM
  TIM4->CR2 = 0x0000;   //Always set to 0 in this course
  TIM4->SMCR = 0x0000;  //Always set to 0 in this course
  TIM4->PSC = 31999;
  TIM4->CNT = 0;
  TIM4->ARR = 0xFFFF;
  TIM4->DIER = 0X0006;  //IRQ activation for CH1 & 2
  TIM4->CCMR1 = 0x0101; //0000 0001 0000 0001
                        //CCyS = 01 (TIC)
                        //OCyCE = 0
                        //OCyM = 000 (no external output)
                        //OCyPE = 0 (no preload)
                        //OCyFE = 0
  TIM4->CCER = 0x0033;  //CCyP = 01 (falling edge)
                        //CCyE = 1 (input capture enabled)
  NVIC->ISER[0] |= (1 << 30); //Enables IRQ source for TIM4 in NVIC (pos 30)

  //FIXME:
  /* TIM2 --------------------------------------------------------------------*/
  //Assigned to PA5 (10 - AFs)
  GPIOA->MODER |= (1 << 5*2 + 1);
  GPIOA->MODER &= ~(1 << 5*2);
  GPIOA->AFR[0] |= (0x1 << 5*4); //0001 - AF1 - TIM2
  //SET-UP for TIM2_CH1 - PWM
  TIM2->CR1 = 0x0080;   //(ARPE=1, CEN=0: timer not counting yet)
  TIM2->CR2 = 0x0000;   //Always set to 0 in this course
  TIM2->SMCR = 0x0000;  //Always set to 0 in this course
  //PSC, CNT, ARR AND CCRx
  TIM2->PSC = 31999;
  TIM2->CNT = 0;
  //TIM2->ARR = X; //Value that when reached by CNT, resets CNT back to 0
  TIM2->CCR1 = duty_cycle;
  TIM2->DIER = 0X0000;
  TIM2->CCMR1 = 0x6800; //0110 1000 0000 0000 meaning
                        //bit15 OC1CE always to 0
                        //bit14-12 OC1M to 110 - PWM with the cycle starting at 1
                        //bi11 OC1PE to 1 - the CCR1 value will be updated after an update event
  TIM2->CCER = 0x0010;  //Enables the hardware output of TIM2_CH1
  //Plus CR1 and EGR to 0x1 to activate timer when needed

  //ADC & Buzzer
  /* ADC_IN4 ------------------------------------------------------------------*/
  //PA4 as analog
  GPIOA->MODER |= (1 << (4*2 + 1));
  GPIOA->MODER |= (1 << (4*2));
  ADC1->CR2 &= ~(0x00000001); // ADON = 0 (ADC powered off initially)
  ADC1->CR1 = 0x00000000;     // OVRIE = 0 (overrun IRQ disabled)
                              // RES = 00 (resolution = 12 bits)
                              // SCAN = 0 (scan mode disabled)
                              // EOCIE = 0 (EOC IRQ disabled)
  ADC1->CR2 = 0x00000412;     // EOCS = 1 (EOC is activated after each conversion)
                              // DELS = 001 (delay till data is read)
                              // CONT = 1 (continuous conversion)
  ADC1->SQR1 = 0x00000000;    // Set to 0 - 1 conversion
  ADC1->SQR5 = 0x00000004;    // Selected first and only conversion is AIN4
  ADC1->CR2 |= 0x00000001;    // ADON = 1 (ADC powered on)

  /* BUZZER -------------------------------------------------------------------*/
  //PA5 as AF because we want it as PWM (send different frequencies at different points)
  GPIOA->MODER |= (1 << (5*2 + 1));
  GPIOA->MODER &= ~(1 << (5*2));
  //AF for TIM2_CH1
  GPIOA->AFR[0] |= (0x02 << (5*4)); // Writes 0010 in AFRL5

  //LEDs
  /* LED1 ---------------------------------------------------------------------*/
  //PA12 (EXTERNAL LED1) - digital output (01)
  GPIOA->MODER &= ~(1 << (12*2 + 1));
  GPIOA->MODER |= (1 << (12*2));
  //Set up with pull-up resistor (01)
  GPIOA->PUPDR &= ~(1 << (12*2 + 1));
  GPIOA->PUPDR |= (1 << (12*2));
  /* LED2 ---------------------------------------------------------------------*/
  //PD2 (EXTERNAL LED2) - digital output (01)
  GPIOD->MODER &= ~(1 << (2*2 + 1));
  GPIOD->MODER |= (1 << (2*2));
  //Set up with pull-up resistor (01)
  GPIOD->PUPDR &= ~(1 << (2*2 + 1));
  GPIOD->PUPDR |= (1 << (2*2));

  /* USART --------------------------------------------------------------------*/
  //RX - PC11 - AF (10)
  GPIOC->MODER |= (1 << (11*2 + 1));
  GPIOC->MODER &= ~(1 << (11*2));
  GPIOA->AFR[1] |= (0x7 << (10*4)); // Writes 0111 in AFRH for pin 11 - USART1_3
                    //change 0x7 bc it probably wont work
  //TX - PC10
  GPIOC->MODER |= (1 << (10*2 + 1));
  GPIOC->MODER &= ~(1 << (10*2));
  GPIOA->AFR[1] |= (0x7 << (10*4)); // Writes 0111 in AFRH for pin 10
  //USART3 registers

  //for USART3
  NVIC->ISER[1] |= (1 << 7); //position 39 (for ISER[1], 0 is 32, 7 is 39)
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Global Requirements
    //Display GAME 1 (initially) --> DONE IN GLOBAL VAR DECLARATION
    //If the USER BUTTON is pressed, change to GAME 2 (at ANY time) --> global if

    //INITIAL SETUP before each game runs
    //All LEDs shall be initially off - when changing game modes, upon a restart of the main while loop
    GPIOA->BSRR = (1 << 12) << 16;
    GPIOD->BSRR = (1 << 2) << 16;
    //Reset counters
    TIM3->CNT = 0;
    TIM4->CNT = 0;
    //Clear all timer flags to be used
    TIM4->SR = 0;
    TIM3->SR = 0;

    //The following global IF condition is used so we may immediately switch between games
    //(a WHILE would force us to finish the nested code execution, then break outside)
    if (prev_game != game)
    {
      prev_game = game;
      switch(game)
      {
        case 1: //GAME 1 - REACTION TIME
          while (game == 1)
          {
            BSP_LCD_GLASS_Clear(); //Clear LCD
            BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME1");
            espera(2*sec);
            if (prev_game != game) break;
            //GAME STARTS HERE
            BSP_LCD_GLASS_Clear(); //Not strictly needed since we are printing the same No. of chars to display
            BSP_LCD_GLASS_DisplayString((uint8_t*)" READY");
            espera(2*sec);
            if (prev_game != game) break;
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)"  GO");

            //Waiting for users to input
            while (winner == 0)
            {
              if (prev_game != game) break;

              //Start counters
              TIM3->CR1 |= BIT_0;   //Set CEN = 1, Starts the counter
              TIM3->EGR |= BIT_0;   //UG = 1 -> Generate an update event to update all registers
              TIM3->SR = 0;         //Clear counter flags
              randn = random_num(0, 4000); //Before 4 secs, at any RANDOM time LED1 will turn ON
              TIM3->CCR1 = randn; //TIM3 will set a flag when it reaches randn

              while((TIM3->SR &0x0002) == 0); //loop until there is an event in timer (finishes counting up to randn)
              TIM3->SR &= ~(0x0002); //Clear flag after event

              //Start counting the time delta to a player pressing their button as LED1 lights up
              TIM4->CR1 |= BIT_0;
              TIM4->EGR |= BIT_0;
              TIM4->SR = 0;

              if (winner == 0) GPIOA->BSRR = (1 << 12); // LED ON while no player has pressed their button yet
              playing = 1; //We only want to accept the button presses after LED1 is lit

              if (prev_game != game) break;
            }

            //WINNER is determined by interrupts, they will change the var winner to 1 or 2 respectively
            if (winner == 1)
            {
              BSP_LCD_GLASS_Clear();

              //Concatenates the player number before reaction time
              //player Y; reaction time XXXXX
              //format for display = YXXXXX
              time_4ch2 += 10000; //In this case = 1XXXXX
              Bin2Ascii(time_4ch2, text); //Transforms the short data type to ASCII
              BSP_LCD_GLASS_DisplayString((uint8_t*) text);

              espera(3*sec); //wait so the player acknowledges their win
              winner = 0; //reset winner for future match
              playing = 0;
            }
            else if (winner == 2) // We use an else if because we only want ONE winner
            {
              BSP_LCD_GLASS_Clear();

              //Concatenates the player number before reaction time
              time_4ch1 += 20000;
              Bin2Ascii(time_4ch1, text);
              BSP_LCD_GLASS_DisplayString((uint8_t*) text);

              espera(3*sec);
              winner = 0;
              playing = 0;
            }
          }
        break;

        case 2: //GAME 2 - COUNTDOWN
            while (game == 2)
            {
              //Users are displayed a countdown in real time from 10 to 0
              //At a random time (a while before 0 - use TIM3 with rand up to 10)
              //the countdown will STOP being displayed
              //The players have to attempt to press the button when the countdown reaches 0

              //The player with the closest time delta to 0 will win, and their LED will light up
              //Pressing before the countdown ends will result in displaying -XXXX time left
              //Pressing after the countdown ends will result in displaying +XXXX time passed
              //The player to have the closest absolute value to 0, wins

              BSP_LCD_GLASS_Clear();
              BSP_LCD_GLASS_DisplayString((uint8_t*)" GAME2");
              espera(2*sec);
              if (prev_game != game) break;

              BSP_LCD_GLASS_Clear();
              BSP_LCD_GLASS_DisplayString((uint8_t*)" READY");
              espera(2*sec);
              if (prev_game != game) break;

              //Initial conditions reset
              winner = 0;
              btn_pressed = 0;
              time_4ch1 = 0;
              time_4ch2 = 0;
              time_2ch1 = 0;
              high_limit_randn = 0;
              timer_count_limit = 0;
              timeout_time = 0;

              while ((ADC1->SR &0x0040) == 0); // If ADCONS = 0, I wait till converter is ready
              ADC1->CR2 |= 0x40000000; // When ADCONS = 1, I start conversion (SWSTART = 1)
              while ((ADC1->SR &0x2) == 0); //While EOC is not flagged, wait
              adc_value = ADC1->DR; //Assign value after successfull conversion
              
              /*
              //Testing check
              Bin2Ascii(adc_value, text);
              BSP_LCD_GLASS_DisplayString((uint8_t*) text);
              espera(2*sec);
              */

              //This strucutre runs before each game setting the countdown mode based on the value read from the potentiometer when starting the game
              x = 1000; //1 volt
              y = 2000; //2 volts
              if (adc_value < x) potentiometer_value = 1;
              else if (adc_value > y) potentiometer_value = 3;
              else potentiometer_value = 2;

              //Start counters
              TIM3->CR1 |= BIT_0;   //Set CEN = 1, Starts the counter
              TIM3->EGR |= BIT_0;   //UG = 1 -> Generate an update event to update all registers
              TIM3->SR = 0;         //Clear counter flags

              switch(potentiometer_value)
              {
                  case 1:
                    high_limit_randn = 4500;
                    timer_count_limit = 5000;
                    timeout_time = 7000;
                  break;
                  case 2:
                    high_limit_randn = 9500;
                    timer_count_limit = 10000;
                    timeout_time = 12000;
                  break;
                  case 3:
                    high_limit_randn = 19500;
                    timer_count_limit = 20000;
                    timeout_time = 22000;
                  break;
              }
              randn = random_num(low_limit_randn, high_limit_randn); //rand num between 0.5 and x seconds
                                //(so the players can see the game started and it doesnt finish too close to 0)
              TIM3->CCR1 = randn;

              TIM4->CR1 |= BIT_0;
              TIM4->EGR |= BIT_0;

              while (btn_pressed == 0)
              {
                while ((TIM3->SR &0x0002) == 0) /*Keep displaying digits while CNT is not reached*/
                {
                  time_3 = TIM3->CNT;
                  countdown = timer_count_limit - time_3;

                  //For checkpoint 3 - a switch that makes the countdown decrease in steps of 0.5s, 1s and 2s (displaying all whole nums: 10, 9, 8...)
                  switch(potentiometer_value)
                  {
                    case 1: //lowest potentiometer position
                      if(countdown%500 != 0) break; // Guard clause - if not met, break, if met, allow folling code to execute 
                      countdown /= 500;
                      Bin2Ascii(countdown, text);
                      BSP_LCD_GLASS_DisplayString((uint8_t*) text);
                      //HAL_UART_Transmit(&huart3, (uint8_t*) text, 6, 10000);
                    break;
                    case 2: //mid pos
                      //To display only whole numbers (1sec increments) if the
                      //number is a multiple of 10000, divide it by 1000 and display
                      //In this case the LCD only display whole numbers (10, 9, 8, etc)
                      if(countdown%1000 != 0) break;
                      countdown /= 1000;
                      Bin2Ascii(countdown, text);
                      BSP_LCD_GLASS_DisplayString((uint8_t*) text);
                      //HAL_UART_Transmit(&huart3, countdown, 6, 10000);
                    break;
                    case 3: //max pos
                      if(countdown%2000 != 0) break;
                      countdown /= 2000;
                      Bin2Ascii(countdown, text);
                      BSP_LCD_GLASS_DisplayString((uint8_t*) text);
                      //HAL_UART_Transmit(&huart3, text, 6, 10000);
                    break;
                  }

                  if (prev_game != game) break;
                }
                if ((TIM3->SR &0x0002) != 0)
                {
                  BSP_LCD_GLASS_Clear(); //Clear LCD when randn is reached by TIM3
                  //We will wait here for up to 2secs after the count passes 10secs while no winner is defined
                  if (TIM3->CNT > timeout_time) break; //if nobody presses any button after 13s the game resets
                }

                if (prev_game != game) break;
              }
              TIM3->SR &= ~(0x0002); //Clear flag after any button press

              //WINNER is determined by the whoever pressed closest to 0
              //when 1st player clicks save their time to a var
              //wait for the second player to click and save to var2
              //compare vars, smallest absolute value wins

              //If the game was engaged with, wait for the other player to attempt their press
              while ((time_4ch2 == 0) || (time_4ch1 == 0))
              {
                if((time_4ch2 == 0) && (time_4ch1 == 0))
                {
                  BSP_LCD_GLASS_DisplayString((uint8_t*) "  END");
                  espera(2*sec);
                  break; //if no one pressed, break
                }
                //Grace period of 2secs from countdown reaching 0, if not automatically the only player to click will win
                if (TIM3->CNT > timeout_time)
                {
                  if (time_4ch1 == 0) //If only P1 pressed, they win
                  {
                    winner = 1;
                    break; //break out of while loop
                  }
                  else if (time_4ch2 == 0) //If only P2 pressed, they win
                  {
                    winner = 2;
                    break;
                  }
                }
              }

              if(time_4ch1 == time_4ch2) //unlikely but maybe still a possibility
              {
                if ((time_4ch1 == 0) && (time_4ch2 == 0)); //no one played - discard
                else
                BSP_LCD_GLASS_DisplayString((uint8_t*) "  TIE");
                espera(2*sec);
              }
              else if((time_4ch1 == 0) || (time_4ch2 == 0))
              {
                //If one of them is 0, with the code above, the only player to press automatically won
                //This if statement is meant as a security measure, we discard that option and continue with the code
              }
              else if(abs(time_4ch1 - timer_count_limit) > abs(time_4ch2 - timer_count_limit)) //who is closer to timer_count_limit ??
              {
                winner = 1;
              }
              else if(abs(time_4ch2 - timer_count_limit) > abs(time_4ch1 - timer_count_limit))
              {
                winner = 2;
              }

              if (winner == 1)
              {
                GPIOA->BSRR = (1 << 12); //Turn on LED1 to indicate P1 won

                //Pressing before the countdown ends will result in displaying -XXXX time left
                //Pressing after the countdown ends will result in displaying +XXXX time passed

                //Depending on the max value we must calculate the time delta
                delta = timer_count_limit - time_4ch2;
                Bin2Ascii(delta, text);
                BSP_LCD_GLASS_DisplayString((uint8_t*) text);
                if(time_4ch2 < timer_count_limit)
                {
                  //TODO: check downloaded code from buzzer/src, stablish functionality and diff freqs.
                  //PLAY MELODY 1 (player pressed before end of countdown)
                  //TIM2->CR1 |= 0x0001; //Timer on toggle
                  //TIM2->EGR |= 0x0001; //Update event
                  BSP_LCD_GLASS_DisplayString((uint8_t*) " -");
                }
                else
                {
                  //PLAY MELODY 2 (player pressed before end of countdown)
                  //TIM2->CR1 |= 0x0001;
                  //TIM2->EGR |= 0x0001;
                  //use different increasing step (double the frequencies to change octave)
                  //time_2ch1 // what??
                  BSP_LCD_GLASS_DisplayString((uint8_t*) " +");
                }

                espera(2*sec); //wait so the player acknowledges their win
                GPIOA->BSRR = (1 << 12) << 16; //Turn off winners LED after win
                TIM2->SR = 0;
              }
              else if (winner == 2)
              {
                GPIOD->BSRR = (1 << 2); //Turn on LED2 to indicate P2 won

                delta = timer_count_limit - time_4ch1;
                Bin2Ascii(delta, text);
                BSP_LCD_GLASS_DisplayString((uint8_t*) text);
                if(time_4ch1 < timer_count_limit)
                {
                  //PLAY MELODY 1
                  //TIM2->CR1 |= 0x0001;
                  //TIM2->EGR |= 0x0001;
                  BSP_LCD_GLASS_DisplayString((uint8_t*) " -");
                }
                else
                {
                  //PLAY MELODY 2
                  //TIM2->CR1 |= 0x0001;
                  //TIM2->EGR |= 0x0001;
                  //different step
                  BSP_LCD_GLASS_DisplayString((uint8_t*) " +");
                }

                espera(2*sec);
                GPIOD->BSRR = (1 << 2) << 16;
                TIM2->SR = 0;
              }
              /** Checkpoint 4
                //USART3 help code

                HAL_UART_Transmit (&huart1,(uint8_t*) data , 5 ,0xFFFF);
                // USER CODE END PV
                HAL functions:
                HAL_UART_Receive_IT(&huart1, (uint8_t*) data_rx , 1); 
                HAL_UART_Receive(&huart1, (uint8_t*) data_rx , 1, 0xFFFF); 
               */

            } //END OF GAME2
        break;

        //This code below should be unreachable on purpose
        default:
          GPIOA->BSRR = (1 << 12) << 16;
          BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t*)" ERROR");
          espera(2*sec);
          BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t*)" RESET");
        break;
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LCD;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  *         where the assert_param error has occurred.
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
