/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcdtp.h"
#include "xpt2046.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_WIDTH 160  //x axis for the machine, vertical page for the LCD
#define FRAME_HEIGHT 20  //y axis for the machine, horizontal column for the LCD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int drawable=0;

char patterns[4][FRAME_WIDTH][FRAME_HEIGHT]={0}; 

enum Page{HOME, CANVA, MACHINE_DRAW, CALIBRATION}; 

//functions for the HOME page
void DisplayHome(){
  LCD_Clear ( 0, 0, 240, 320, WHITE );

  LCD_Clear ( 90,  70,  60, 60, GREEN);
  LCD_DrawString(90, 70, "MACHINE DRAW");

  LCD_Clear ( 90,  150,  60, 60, RED);
  LCD_DrawString(90, 150, "CALIBRATE");

  LCD_Clear ( 90,  230,  60, 60, BLUE);
  LCD_DrawString(90, 230, "CANVA");

}

//functions for pages to return to home
void check_homeKey(enum Page* page_ptr){
  GPIO_PinState k2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  if ((k2==GPIO_PIN_SET)){
	  *page_ptr = HOME;
    DisplayHome();
    HAL_Delay(500);
  }
}

//functions for the CALIBRATION page
void DisplayCalibration(){
  LCD_Clear ( 0, 0, 240, 320, WHITE );
  //x, y, penUp/ penDown, rotate 
  //x coords: 30-60, 90, 120-150
  //y coords: 10
  LCD_DrawString(50, 10, "-ve");
  LCD_DrawString(140, 10, "+ve");
  LCD_DrawString(110, 60, "X"); 
  LCD_DrawString(110, 110, "Y"); 
  LCD_DrawString(110, 170, "pen");
  LCD_DrawString(110, 220, "rotate: pen UP");

//x
  LCD_Clear ( 40,  40,  30, 30, RED);
  LCD_Clear ( 150,  40,  30, 30, RED);

  //y
  LCD_Clear ( 40,  90,  30, 30, BLUE);
  LCD_Clear ( 150,  90,  30, 30, BLUE);

  //pen
  LCD_Clear ( 40,  150,  30, 30, GREEN);
  LCD_Clear ( 150,  150,  30, 30, GREEN);

  //rotate
  LCD_Clear ( 40,  200,  30, 30, YELLOW);

}


//functions for the CANVA page 

void DrawRedBox(uint16_t startP, uint16_t startC) {
    uint16_t endP = startP+FRAME_WIDTH*2-1; // X coordinate of the bottom-right corner
    uint16_t endC = startC+ FRAME_HEIGHT*2-1 ; // Y coordinate of the bottom-right corner

    // Draw the four sides of the box
    LCD_DrawLine(startC, startP, endC, startP, RED); // Top side
    LCD_DrawLine(startC, startP, startC, endP, RED); // Left side
    LCD_DrawLine(endC, startP, endC, endP, RED); // Right side
    LCD_DrawLine(startC, endP, endC, endP, RED); // Bottom side
}

void DisplayCanva(){
	LCD_Clear ( 0, 0, 240, 320, WHITE );
				DrawRedBox(0, 10);
				  DrawRedBox(0, 70);
				  DrawRedBox(0, 130);
				  DrawRedBox(0, 190);
}

// startP and startC are the top-left corner of the box
//pattern will be saved into patterns[pattern_num]
//page corresponds to FRAME_WIDTH, x axis 
//column corresponds to FRAME_HEIGHT, y axis 
void savePattern(uint16_t startP, uint16_t startC, int pattern_num) {
    for (int i = 0; i < FRAME_WIDTH; i++) {
        for (int j = 0; j < FRAME_HEIGHT; j++) {
          int black_count=0; 
          black_count += (LCD_GetPointPixel(startC+j*2, startP+i*2)==BLACK);
          black_count += (LCD_GetPointPixel(startC+j*2+1, startP+i*2)==BLACK);
          black_count += (LCD_GetPointPixel(startC+j*2, startP+i*2+1)==BLACK);
          black_count += (LCD_GetPointPixel(startC+j*2+1, startP+i*2+1)==BLACK);
          patterns[pattern_num][i][j] = (black_count>=2);
        }
    }
}

void displayPattern(uint16_t startP, uint16_t startC, int pattern_num, uint16_t colour){

	for (int i = 0; i < FRAME_WIDTH; i++) {
		        for (int j = 0; j < FRAME_HEIGHT; j++) {
		          if(patterns[pattern_num][i][j]){
		            LCD_Clear(startC+j*2, startP+i*2, 2, 2, colour);
		          }
		        }
		        }

}

void consolidatePattern(){
  savePattern(0, 10, 0); 
  savePattern(0, 70, 1);
  savePattern(0, 130, 2);
  savePattern(0, 190, 3);
  DisplayCanva();
  displayPattern(0, 10, 0, RED);
  displayPattern(0, 70, 1, RED);
  displayPattern(0, 130, 2, RED);
  displayPattern(0, 190, 3, RED);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin==GPIO_PIN_0){
		if (drawable){
			consolidatePattern();
			drawable =0; //returns control to main
		}
		else if (!drawable){
			DisplayCanva();
			  drawable =1; //goes to exti 4 irq
		}
		//drawable= (drawable+1)%2;
	}
}

//funtion for the MACHINE_DRAW page

void DisplayMachineDraw(){
  LCD_Clear ( 0, 0, 240, 320, WHITE );
  }




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
  MX_GPIO_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
	
	macXPT2046_CS_DISABLE();
	
	LCD_INIT();

	LCD_Clear (50, 80, 140, 70, RED);
	LCD_DrawString(68, 100, "TOUCHPAD DEMO");
	HAL_Delay(2000);

	while( ! XPT2046_Touch_Calibrate () );
  enum Page current_page = HOME;
  DisplayHome();
	drawable=0;

	//DisplayCanva();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
    if (current_page==HOME){
      
	
    if ( ucXPT2046_TouchFlag == 1 ){
      //Check_touchkey();
       strType_XPT2046_Coordinate strDisplayCoordinate;
       if ( XPT2046_Get_TouchedPoint ( & strDisplayCoordinate, & strXPT2046_TouchPara ) ){
        //check which button is pressed

        //canva
         if (strDisplayCoordinate.x>90 && strDisplayCoordinate.x<150 && strDisplayCoordinate.y>230 && strDisplayCoordinate.y<290){
        	 LCD_Clear (0,  0,  240, 320, WHITE); //don't know why need this
        	 current_page = CANVA;
           DisplayCanva();
		      drawable =1; //goes to exti 4 irq
         }
        //calibrate 
         else if (strDisplayCoordinate.x>90 && strDisplayCoordinate.x<150 && strDisplayCoordinate.y>150 && strDisplayCoordinate.y<210){
        	 LCD_Clear (0,  0,  240, 320, WHITE); //don't know why need this
        	 current_page = CALIBRATION;
          DisplayCalibration();
         }
        //machine draw
         else if (strDisplayCoordinate.x>90 && strDisplayCoordinate.x<150 && strDisplayCoordinate.y>70 && strDisplayCoordinate.y<130){
        	 LCD_Clear (0,  0,  240, 320, WHITE); //don't know why need this
        	 current_page = MACHINE_DRAW;
          DisplayMachineDraw();
         }
       }
      ucXPT2046_TouchFlag = 0;
    }

    }

    if (current_page==CANVA){
      LCD_DrawString(68, 100, "main");
      check_homeKey(&current_page); 

    }

    if (current_page==CALIBRATION){
      if ( ucXPT2046_TouchFlag == 1 ){
     
       strType_XPT2046_Coordinate strDisplayCoordinate;
       if ( XPT2046_Get_TouchedPoint ( & strDisplayCoordinate, & strXPT2046_TouchPara ) ){
        //check which button is pressed

        //x -ve
         if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>40 && strDisplayCoordinate.y<70){
        	 //x -ve is pressed
           LCD_DrawString(110, 250, "x -ve");
         }
        //x +ve
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>40 && strDisplayCoordinate.y<70){
        	//x +ve is pressed
          LCD_DrawString(110, 250, "x +ve");
         }
        //y -ve
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>90 && strDisplayCoordinate.y<120){
        	//y -ve is pressed
          LCD_DrawString(110, 250, "y -ve");
         }
        //y +ve
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>90 && strDisplayCoordinate.y<120){
         //y +ve
         LCD_DrawString(110, 250, "y +ve");
         }
        //pen down
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>150 && strDisplayCoordinate.y<180){
             //pen down
            LCD_DrawString(110, 250, "pen down");
         }
        //pen up
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>150 && strDisplayCoordinate.y<180){
            //penup
            LCD_DrawString(110, 250, "pen up");
         }
        //rotate
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>200 && strDisplayCoordinate.y<230){
            //maybe force pen up 
            //rotate
            LCD_DrawString(110, 250, "rotate");
         }
       }
      ucXPT2046_TouchFlag = 0;
    }

      check_homeKey(&current_page); 

    }
    if (current_page==MACHINE_DRAW){
      check_homeKey(&current_page); 
    }
	  
//    if ( ucXPT2046_TouchFlag == 1 )
//    {
//			Check_touchkey();
////	strType_XPT2046_Coordinate strDisplayCoordinate;
////	if ( XPT2046_Get_TouchedPoint ( & strDisplayCoordinate, & strXPT2046_TouchPara ) ){
////		LCD_DrawDot(strDisplayCoordinate.x, strDisplayCoordinate.y, BLACK);
////	}
//      ucXPT2046_TouchFlag = 0;
//    }

//	  
//	  
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//		  HAL_Delay(500);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);
//		  HAL_Delay(500);
		  //consolidatePattern();
//		  LCD_Clear (50, 80, 140, 70, RED);
//		  HAL_Delay(500);
//		  LCD_Clear (50, 80, 140, 70, WHITE);
//
//	  }


    
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : K2_Pin */
  GPIO_InitStruct.Pin = K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(K2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : K1_Pin */
  GPIO_InitStruct.Pin = K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(K1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
