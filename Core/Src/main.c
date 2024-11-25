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
#include <stdlib.h>
#include <stdio.h>

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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int drawable=0;

char patterns[4][FRAME_HEIGHT][FRAME_WIDTH]={0}; 

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
          patterns[pattern_num][j][i] = (black_count>=2);
        }
    }
}

void displayPattern(uint16_t startP, uint16_t startC, int pattern_num, uint16_t colour){

	for (int i = 0; i < FRAME_WIDTH; i++) {
		        for (int j = 0; j < FRAME_HEIGHT; j++) {
		          if(patterns[pattern_num][j][i]){
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

//helper functions
void printline(char* line, int* row){
  if (*row>300){
    LCD_Clear(0,60,240,260, WHITE);
    *row=60;
  }
  LCD_DrawString(0, *row, line);
  *row+=20;
}

void machine_draw_displayPattern(char pattern[FRAME_HEIGHT][FRAME_WIDTH], uint16_t startP, uint16_t startC){
  LCD_Clear(startC, startP, FRAME_WIDTH, FRAME_HEIGHT, WHITE);
  LCD_DrawLine(startC, startP, startC+FRAME_WIDTH, startP, BLACK);
  LCD_DrawLine(startC, startP, startC, startP+FRAME_HEIGHT, BLACK);
  LCD_DrawLine(startC+FRAME_WIDTH, startP, startC+FRAME_WIDTH, startP+FRAME_HEIGHT, BLACK);
  LCD_DrawLine(startC, startP+FRAME_HEIGHT, startC+FRAME_WIDTH, startP+FRAME_HEIGHT, BLACK);
  for (int i = 0; i < FRAME_WIDTH; i++) {
    for (int j = 0; j < FRAME_HEIGHT; j++) {
      if(pattern[j][i]){
        LCD_DrawDot(startC+i, startP+j, BLACK);
      }
    }
  }
}

//fucntions for the machine

struct point {
  int x; //correspond to FRAME_WIDTH
  int y; //correspond to FRAME_HEIGHT
  int a; //angle
};

//stepper motor

enum MotorType{
  M_42BYG,
  M_28BYJ
};

struct StepperMotor{
  GPIO_TypeDef *gpio_port_in1; 
  uint16_t gpio_pin_in1;
  GPIO_TypeDef *gpio_port_in2; 
  uint16_t gpio_pin_in2;
  GPIO_TypeDef *gpio_port_in3;
  uint16_t gpio_pin_in3;
  GPIO_TypeDef *gpio_port_in4;
  uint16_t gpio_pin_in4;
  enum MotorType motor_type;
  int last_step_num; //only for a_motor
};

void delay (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/* use like delay */

void stepper_set_rpm (int rpm, int stepsperrev)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
  //microseconds     rev        minute
  //------------ x --------- x --------
  //    minute        steps      rev
}

void stepper_28byj_half_drive (int step, struct StepperMotor *m )
{
	switch (step){
		case 0:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 4:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 5:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
			  break;

		case 6:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
			  break;

		case 7:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
			  break;

		}
}

void stepper_42byg_full_drive (int step, struct StepperMotor* m)//for 42byg //full drive
{

	switch (step){
		case 0:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3,GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4,GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
			  break;
	
  }
  
}

void stepper_42byg_half_drive(int step, struct StepperMotor* m) // for 42byg // half drive
{
    switch (step) {
        case 0:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET); // IN4
            break;

        case 1:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET); // IN4
            break;

        case 2:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET); // IN4
            break;

        case 3:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET); // IN4
            break;

        case 4:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_RESET); // IN4
            break;

        case 5:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
            break;

        case 6:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
            break;

        case 7:
            HAL_GPIO_WritePin(m->gpio_port_in1, m->gpio_pin_in1, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(m->gpio_port_in2, m->gpio_pin_in2, GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(m->gpio_port_in3, m->gpio_pin_in3, GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(m->gpio_port_in4, m->gpio_pin_in4, GPIO_PIN_SET);   // IN4
            break;
    }
}

void stepper_step_angle (float angle, int direction, int rpm, struct StepperMotor* m) 
{
  //float anglepersequence = 
  float anglepersequence ; 
  if (m->motor_type == M_42BYG){
    anglepersequence = 7.2; //360/(200/4), or simply 1.8*4

  }
  else if( m->motor_type == M_28BYJ){
    anglepersequence = 0.703125;  // 360 = 512 sequences
    //512= 4096/8
  }

  int numberofsequences = (int) (angle/anglepersequence);


  if (m->motor_type == M_42BYG){
      for (int seq=0; seq<numberofsequences; seq++){
    // if (direction == 0){  // for clockwise
    //   for (int step=0; step<4; step++){
    //     stepper_42byg_full_drive(step, m);
    //   stepper_set_rpm(rpm, 200);
    //   }

    // }

    // else if (direction == 1){  // for anti-clockwise
    //   for (int step=3; step>=0; step--)
    //   {
    //     stepper_42byg_full_drive(step, m);
    //   stepper_set_rpm(rpm, 200);
    //   }
    // }

    if (direction == 0){  // for clockwise
      for (int step=0; step<8; step++){
        stepper_42byg_half_drive(step, m);
      stepper_set_rpm(rpm, 400);
      }

    }

    else if (direction == 1){  // for anti-clockwise
      for (int step=7; step>=0; step--)
      {
        stepper_42byg_half_drive(step, m);
      stepper_set_rpm(rpm, 400);
      }
    }
    }
  }
  else if (m->motor_type == M_28BYJ){
    for (int seq=0; seq<numberofsequences; seq++){
      if (direction == 0)  // for clockwise
    {
      for (int step=7; step>=0; step--)
      {
        stepper_28byj_half_drive(step, m);
        stepper_set_rpm(rpm, 4096);
      }

    }

    else if (direction == 1)  // for anti-clockwise
    {
      for (int step=0; step<8; step++)
      {
        stepper_28byj_half_drive(step, m);
        stepper_set_rpm(rpm, 4096);
      }
    }
    }
  }
}

//define the motor as global var
struct StepperMotor m28byj={GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, M_28BYJ};
struct StepperMotor x_motor={GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_4, M_42BYG, 0};
struct StepperMotor a_motor={GPIOD, GPIO_PIN_2, GPIOC, GPIO_PIN_12, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7, M_42BYG, -1};
//FIXME: make the last_step_num more consistent.
//rn, last_step_num is actually next step (the step is used first, then updated) in the x axis
//really last step for a axis (update first, then use the step )

//scale: 60 deg for 1 mm
//60/ 0.9= 66.67. take 67
void x_pos_one_mm(struct StepperMotor* m, int rpm){ //for x axis only
	for (int i=0; i<67; i++){
			stepper_42byg_half_drive(m->last_step_num, m);
			stepper_set_rpm(rpm, 400);
			m->last_step_num= (m->last_step_num+1)%8;
		}
}

void x_neg_one_mm(struct StepperMotor* m, int rpm){ //for x axis only
	for (int i=0; i<67; i++){
			stepper_42byg_half_drive(m->last_step_num, m);
			stepper_set_rpm(rpm, 400);
			m->last_step_num= (m->last_step_num-1);
			if (m->last_step_num<0){
				m->last_step_num+=8;
			}
		}
}

void move_x(int dist){ //in terms of mm
	int dir;
	int rpm=50;
	if (dist>0){
		dir= 0; //to the left //away from motor
		for (int i=0; i<abs(dist); i++){
			x_pos_one_mm(&x_motor, rpm);
		}


	}
	else if (dist<0){ 
		dir=1; //to the right //toward motor
		for (int i=0; i<abs(dist); i++){
			x_neg_one_mm(&x_motor, rpm);
		}

	}

//	for (int i=0; i<abs(dist); i++){
//		stepper_step_angle(45, dir,rpm, &x_motor);
//	}
}

void move_y(int dist){ //in terms of mm
	int dir;
	int rpm=12;
	if (dist<0){
			dir= 0; //towards motor
		}
		else if (dist>0){
			dir=1; //away from motor
		}

		for (int i=0; i<abs(dist); i++){
			stepper_step_angle (360, dir, rpm, &m28byj);
		}
}

void rotate_90(struct StepperMotor* m){
	for (int i=0; i<100; i++){
		m->last_step_num= (m->last_step_num+1)%8;
		stepper_42byg_half_drive(m->last_step_num, m);
		stepper_set_rpm(70, 400);
	}
}



void penUp(int* row){
  //char buffer[20];
  //sprintf(buffer, "Pen up");
  //printline(buffer, row);
  TIM2->CCR3= 500;
}

void penDown(int* row){
  //char buffer[20];
  //sprintf(buffer, "Pen down");
  //printline(buffer, row);

    TIM2->CCR3= 1000;
}

void rotate_fn(struct point newPos, struct point* actuatorPos, int* row){
  rotate_90(&a_motor); 
  char buffer[40];
  sprintf(buffer, "Rotate %d to %d", actuatorPos->a,newPos.a);
  printline(buffer, row);
  actuatorPos->a = newPos.a; 
  }; 

void drawLine(struct point newPos, struct point* actuatorPos, int* row){
  int x_diff= newPos.x - actuatorPos->x;
  int y_diff= newPos.y - actuatorPos->y;
  char buffer[40];
  sprintf(buffer, "From [%d, %d] to [%d, %d]", actuatorPos->y, actuatorPos->x, newPos.y, newPos.x);
  printline(buffer, row);
  move_x(x_diff);
  move_y(y_diff);
  actuatorPos->x = newPos.x;
  actuatorPos->y = newPos.y;
}

void reset_home(struct point* actuatorPos, int* row){
  move_x(-actuatorPos->x);
  move_y(-actuatorPos->y);
  actuatorPos->x = 0;
  actuatorPos->y = 0;
  char buffer[40];
  sprintf(buffer, "Reset home");
  printline(buffer, row);
}


void DisplayMachineDraw(){
  LCD_Clear ( 0, 0, 240, 320, WHITE );
  }


//return -1 if no such bit found
//value is either 0 or 1 
int find_next_start(char arr[FRAME_HEIGHT][FRAME_WIDTH], int row, int start_index, int dir){ 
    int position= start_index; 
    if (dir==1){
        for (;position<FRAME_WIDTH && arr[row][position]!=1; position++);  
                   //case 0, no such bit found, whole line empty
                   //then position will be FRAME_WIDTH
                   //case 1: bit is found. then  arr[row][position]==1
        if (position==FRAME_WIDTH){
            return -1; 
        }
        return position;
    }
    else if (dir==-1){
        for (;position>=0 && arr[row][position]!=1; position--);  
                   //case 0, no such bit found, whole line empty
                   //then position <0
        if (position<0){
            return -1; 
        }
        return position;
    }
    return -1;
}

int find_next_end(char arr[FRAME_HEIGHT][FRAME_WIDTH], int row, int start_index, int dir){ 
    int position= start_index; 
    if (dir==1){
        for (;position<FRAME_WIDTH && arr[row][position]!=0; position++);  
                //case 0, no such bit found, whole line full i.e.arr[row][FRAME_WIDTH-1]==1
                //then position will be FRAME_WIDTH, we should return FRAME_WIDTH-1
                //case 1: bit is found. then  arr[row][position]==0, we should return position-1
        position= (position < 0)? 0: position-1; //hopefully we never get position<0...
        return position;
    }
    else if (dir==-1){
        for (;position>=0 && arr[row][position]!=0; position--);  
                   //case 0, no such bit found, whole line full i.e.arr[row][0]==1
                   //but now position will be -1, we should return 0
                //case 1: bit is found. then  arr[row][position]==0, we should return position+1
        position= (position> FRAME_WIDTH-1)? FRAME_WIDTH-1: position+1; //hopefully we never get position>FRAME_WIDTH-1...
        return position;
    }
    return -1; //hopefully we never reach here 
}


void draw_plane(char arr[FRAME_HEIGHT][FRAME_WIDTH], struct point* actualPos, int* row){//1: draw. 0: don't draw

    int scan_dir=1; //1 means left2right. -1 means right2left
    int start_pos=0; 
    int end_pos =0 ;
    for (int r=0; r<FRAME_HEIGHT; r++){

        if (scan_dir==1){
            start_pos=0; 
            end_pos =0 ;  //line is [start_pos, end_pos]   
        }
        else if (scan_dir==-1){
            start_pos=FRAME_WIDTH-1 ; 
            end_pos =FRAME_WIDTH-1  ;}  //line is [start_pos, end_pos]
        while ((scan_dir==1 && end_pos<FRAME_WIDTH-1)|| (scan_dir==-1 && end_pos>0)){
            //scan dir ==1
            //case 0, line [FRAME_WIDTH-1 ] is 0, then the start_pos will eventually be -1
            //case 1: line[FRAME_WIDTH-1] is 1, then the end_pos will be FRAME_WIDTH-1 eventually
            
            //scan dir ==-1 
            //case 0: line[0] is 0, then the start_pos will eventually be -1
            //case 1: line[0] is 1, then the end_pos will be 0 eventually 
            start_pos= find_next_start(arr, r, start_pos, scan_dir); 
            //printf("next start: %d", start_pos); 
            if (start_pos==-1){ //remaining line empty
                break; //move on to next line
            }
            //else find the end of the line segment
            end_pos= find_next_end(arr, r, start_pos, scan_dir); 
            struct point newPos= {start_pos, r,actualPos->a}; 
            drawLine(newPos, actualPos, row);  //move to start pos 

            penDown(row); //start drawing
            newPos.x = end_pos; 
            // char buffer[40];
            // sprintf(buffer, "from [%d, %d] to [%d, %d]", actualPos->y, actualPos->x, newPos.y, newPos.x );
            // printline(buffer, row); 

            //visualize drawing //+5 is the offset 
            LCD_DrawLine(actualPos->x+5, actualPos->y+5, newPos.x+5, newPos.y+5, RED); 

            drawLine(newPos, actualPos, row);// actualPos is updated to newPos
            penUp(row);
            

            //end of segment
            if (scan_dir==1){
                 start_pos = end_pos+1; 
                if (start_pos>= FRAME_WIDTH){
                    break; 
                }
                }
            else if (scan_dir==-1){
                start_pos = end_pos-1; 
                if (start_pos< 0){
                    break; 
                }
            }
            }
            
           
        scan_dir *= -1; 
        //move to next line //FIXME: if next line is empty, no need to change dir
        //FIXME: changing line issue

    }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	macXPT2046_CS_DISABLE();
	
	LCD_INIT();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	LCD_Clear (50, 80, 140, 70, RED);
	LCD_DrawString(68, 100, "TOUCHPAD DEMO");
	HAL_Delay(2000);

	while( ! XPT2046_Touch_Calibrate () );
  enum Page current_page = HOME;
  DisplayHome();
	drawable=0;
  int machine_draw_flag=0; 
  struct point actualPos= {0, 0, 0}; 
  int row=60; //start at 60
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
          machine_draw_flag=1;
          //TODO: go home fn //reset the position
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
           move_x(-10);
           
         }
        //x +ve
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>40 && strDisplayCoordinate.y<70){
        	//x +ve is pressed
          LCD_DrawString(110, 250, "x +ve");
          move_x(10);
          
         }
        //y -ve
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>90 && strDisplayCoordinate.y<120){
        	//y -ve is pressed
           LCD_DrawString(110, 250, "y -ve");
          move_y(-1);
         
         }
        //y +ve
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>90 && strDisplayCoordinate.y<120){
         //y +ve
         LCD_DrawString(110, 250, "y +ve");
          move_y(1);
         
         }
        //pen down
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>150 && strDisplayCoordinate.y<180){
             //pen down
            LCD_DrawString(110, 250, "pen down");
            penDown(&row);
         }
        //pen up
         else if (strDisplayCoordinate.x>150 && strDisplayCoordinate.x<180 && strDisplayCoordinate.y>150 && strDisplayCoordinate.y<180){
            //penup
            LCD_DrawString(110, 250, "pen up");
            penUp(&row);
            
         }
        //rotate
         else if (strDisplayCoordinate.x>40 && strDisplayCoordinate.x<70 && strDisplayCoordinate.y>200 && strDisplayCoordinate.y<230){
            //maybe force pen up 
            //rotate
            LCD_DrawString(110, 250, "rotate");
            rotate_90(&a_motor);
            
         }
       }
      ucXPT2046_TouchFlag = 0;
    }

      check_homeKey(&current_page); 

    }
    if (current_page==MACHINE_DRAW){
      if (machine_draw_flag){
        for (int i=0; i<4; i++){
          uint16_t startP= 5; 
          uint16_t startC= 5;
          machine_draw_displayPattern(patterns[i], startP, startC);
          draw_plane(patterns[i], &actualPos, &row);
          penUp(&row);
          //reset_home(&actualPos, &row); //no need to go home
          int new_a= (actualPos.a+90)%360;
          struct point newPos= {0, 0, new_a};
          rotate_fn(newPos, &actualPos, &row);
          
        }
        reset_home(&actualPos, &row); //go home at the end 
        machine_draw_flag=0;
        char buffer[40];
        sprintf(buffer, "Drawing finished. Press K2.");
        printline(buffer, &row);
      }

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOA, X_Pin|XA3_Pin|XA4_Pin|XA5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, board_LED_Pin|board_LEDB1_Pin|board_LEDB5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, y_Pin|yB13_Pin|yB14_Pin|yB15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|AD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A_Pin|AC7_Pin|AC12_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : X_Pin XA3_Pin XA4_Pin XA5_Pin */
  GPIO_InitStruct.Pin = X_Pin|XA3_Pin|XA4_Pin|XA5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : board_LED_Pin board_LEDB1_Pin board_LEDB5_Pin */
  GPIO_InitStruct.Pin = board_LED_Pin|board_LEDB1_Pin|board_LEDB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : y_Pin yB13_Pin yB14_Pin yB15_Pin */
  GPIO_InitStruct.Pin = y_Pin|yB13_Pin|yB14_Pin|yB15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 AD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|AD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin AC7_Pin AC12_Pin */
  GPIO_InitStruct.Pin = A_Pin|AC7_Pin|AC12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
