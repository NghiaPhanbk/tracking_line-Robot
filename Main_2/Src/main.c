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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim ->Instance == TIM1)
//	{
//		count = __HAL_TIM_GetCounter(&htim2);
//		__HAL_TIM_SetCounter(&htim2, 0);
//	}
//}
int count = 0;
uint16_t u16_ADCSensor[7];
uint16_t val1 = 0;	// Counter value when ultrasonic generates wave
uint16_t val2 = 0;	// Counter value when sensor receive echo wave
uint16_t dis = 0;		// Raw distance
float e1 = 0.05;
float T_s=0.05;
float kp_r=0.6, ki_r=8, kd_r=0.0027, pwm_r=0.0;
float kp_f=0.05, ki_f=0.1, kd_f=0.0027;
double error_r=0,sumError_r=0,rateError_r=0,lastError_r=0,speed_r=0,count_r=0;
float kp_l=0.633, ki_l=9.4, kd_l=0.0046, pwm_l=0.0;
double error_l=0,sumError_l=0,rateError_l=0,lastError_l=0,speed_l=0,count_l=0;
float kp_gr=0.2, ki_gr=0.1, kd_gr=0.0027,pwm_gr=0.0;
double error_gr=0,sumError_gr=0,rateError_gr=0,lastError_gr=0,angler=0,count_gr=0;
float kp_gl=0.2, ki_gl=0.1, kd_gl=0.0027,pwm_gl=0.0;
double error_gl=0,sumError_gl=0,rateError_gl=0,lastError_gl=0,anglel=0,count_gl=0;
float nume2=0.0, dene2=0.0, e2=0.0, nume3=0.0,e3=0.0,e2pre=0.0;
float k_x=0.05,k_y=0.01,k_theta=0.01;
//float v_max = 0.5, w_max=10;
float v_max = 0.3, w_max=10;
float vwl=0.0, vwr=0.0;
float angl=0.0, angr=0.0;
float b=0.035, r=0.0425, pi=3.14;
unsigned long currentTime_r, previousTime_r;
double elapsedTime_r ;
unsigned long currentTime_l, previousTime_l;
double elapsedTime_l ;
unsigned long currentTime_gr, previousTime_gr;
double elapsedTime_gr ;
unsigned long currentTime_gl, previousTime_gl;
double elapsedTime_gl ;
unsigned long currentTime, previousTime;
double elapsedTime ;
//uint16_t check_line;
double v_r=0.0,w_r=0,v_c=0,w_c=0;
double set_vwl=0.0,set_vwr=0;
float k=100;
int state=0;
char TxBuff[20];
char RxBuff[20];
uint8_t RxData;
uint8_t rxindex;
uint16_t start;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t u16_ADCSensor[7];;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
UNUSED(huart);
	if (huart->Instance == USART1){
	if(RxData != 13){
	RxBuff[rxindex++]=RxData;
	}
	else if(RxData == 13){
	rxindex=0;	
	}
	else if (RxData == 49){
	start =1;
	}	
	HAL_UART_Receive_IT(&huart1,&RxData,1);
	}
}
void print_data() {
	sprintf(TxBuff, "%s", "DATA, TIME, TIMER, DATE,");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%f", e2);
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%s", ",");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%f", e3);
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%s", ",\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%s", "e3:");
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%f", e3);
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);	
//	sprintf(TxBuff, "%s", ";");	
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%s", "vwr:");
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%f", set_vwr);
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%s", ";");	
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%s", "vwl:");
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
//	sprintf(TxBuff, "%f", set_vwl);
//	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);	
 }
// Sieu am
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_5){
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
			val1 = __HAL_TIM_GET_COUNTER(&htim4);
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0){
			val2 = __HAL_TIM_GET_COUNTER(&htim4);
		}
	}
}

void delay_us(uint32_t us){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (!(__HAL_TIM_GET_COUNTER(&htim4) < us));
}

void trig_hc04(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
}

void read_hc04(){
	dis = (val2 - val1) * 0.034/2;
	}
int signnum_typical(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

void motor(float set_vwr,float set_vwl){
	currentTime_r = HAL_GetTick();
				elapsedTime_r = (double)(currentTime_r - previousTime_r)/1000.0; 
				count_r = __HAL_TIM_GET_COUNTER(&htim2);
				if (count_r>32767) count_r=count_r-65535;
//				__HAL_TIM_SET_COUNTER(&htim2, 0);
				speed_r = (float)(count_r) * 60000.0/ 374.0/4.0/30.0;
				error_r = vwr - speed_r;                                // determine error
        sumError_r += error_r * elapsedTime_r;                // compute integral
        rateError_r = (error_r - lastError_r)/elapsedTime_r;   // compute derivative
        pwm_r = kp_r*error_r + ki_r*sumError_r + kd_r*rateError_r;                //PID output               
        lastError_r = error_r;	
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				previousTime_r = currentTime_r;   
				if(pwm_r >100)
				pwm_r = 100; 
				if(pwm_r <-100)
				pwm_r = -100;
				if(pwm_r>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR1 = (int)pwm_r;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR1 = (int)(-pwm_r);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	
				 }				
				currentTime_l = HAL_GetTick();
				elapsedTime_l = (double)(currentTime_l - previousTime_l)/1000.0; 
				count_l = __HAL_TIM_GET_COUNTER(&htim3);
				if (count_l>32767) count_l=count_l-65535;
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				speed_l = (float)(count_l) * 60000.0/ 374.0/4.0/30.0;
				error_l = vwl - speed_l;                                // determine error
        sumError_l += error_l * elapsedTime_l;                // compute integral
        rateError_l = (error_l - lastError_l)/elapsedTime_l;   // compute derivative
        pwm_l = kp_l*error_l + ki_l*sumError_l + kd_l*rateError_l;                //PID output               
        lastError_l = error_l;	
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				previousTime_l = currentTime_l;   
				if(pwm_l >100)
				pwm_l = 100; 
				if(pwm_l <-100)
				pwm_l = -100;
				if(pwm_l>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR3 = (int)pwm_l;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR3 = (int)(-pwm_l);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	
				 }			
				 HAL_Delay(29);
				 vwr=set_vwr;
				 vwl=set_vwl;
	}
void rotate(float set_angler,float set_anglel){
				currentTime_gr = HAL_GetTick();
				elapsedTime_gr = (double)(currentTime_gr - previousTime_gr)/1000.0; 
				count_gr = __HAL_TIM_GET_COUNTER(&htim2);
				if (count_gr>32767) count_gr=count_gr-65535;
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				angler = (float)(count_gr) * 360/374.0/4.0;
				error_gr = angr - angler;                                // determine error
        sumError_gr += error_gr * elapsedTime_gr;                // compute integral
        rateError_gr = (error_gr - lastError_gr)/elapsedTime_gr;   // compute derivative
        pwm_gr = kp_gr*error_gr + ki_gr*sumError_gr + kd_gr*rateError_gr;                //PID output               
        lastError_gr = error_gr;	
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				previousTime_gr = currentTime_gr;   
				if(pwm_gr >100)
				pwm_gr = 100; 
				if(pwm_gr <-100)
				pwm_gr = -100;
				if(pwm_gr>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR1 = (int)pwm_gr;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR1 = (int)(-pwm_gr);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	
				 }	
				 // PID motor left
				currentTime_gl = HAL_GetTick();
				elapsedTime_gl = (double)(currentTime_gl - previousTime_gl)/1000.0; 
				count_gl = __HAL_TIM_GET_COUNTER(&htim3);
				if (count_gl>32767) count_gl=count_gl-65535;
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				anglel = (float)(count_gl) * 360/374.0/4.0;
				error_gl = angl - anglel;                                // determine error
        sumError_gl += error_gl * elapsedTime_gl;                // compute integral
        rateError_gl = (error_gl - lastError_gl)/elapsedTime_gl;   // compute derivative
        pwm_gl = kp_gl*error_gl + ki_gl*sumError_gl + kd_gl*rateError_gl;                //PID output               
        lastError_gl = error_gl;	
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				previousTime_gl = currentTime_gl;   
				if(pwm_gl >100)
				pwm_gl = 100; 
				if(pwm_gl <-100)
				pwm_gl = -100;
				if(pwm_gl>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR3 = (int)pwm_gl;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR3 = (int)(-pwm_gl);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	
				 }			
				 HAL_Delay(49);
				 angr=set_angler;
				 angl=set_anglel;
				 
	}

	// Di chuyen thang qua vat can
void forward(float set_angler,float set_anglel){
				//PID motor right
				currentTime_gr = HAL_GetTick();
				elapsedTime_gr = (double)(currentTime_gr - previousTime_gr)/1000.0; 
				count_gr = __HAL_TIM_GET_COUNTER(&htim2);
				if (count_gr>32767) count_gr=count_gr-65535;
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				angler = (float)(count_gr) * 360/374.0/4.0;
				error_gr = angr - angler;                                // determine error
        sumError_gr += error_gr * elapsedTime_gr;                // compute integral
        rateError_gr = (error_gr - lastError_gr)/elapsedTime_gr;   // compute derivative
        pwm_gr = kp_f*error_gr + ki_f*sumError_gr + kd_f*rateError_gr;                //PID output               
        lastError_gr = error_gr;	
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				previousTime_gr = currentTime_gr;   
				if(pwm_gr >100)
				pwm_gr = 100; 
				if(pwm_gr <-100)
				pwm_gr = -100;
				if(pwm_gr>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR1 = (int)pwm_gr;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR1 = (int)(-pwm_gr);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	
				 }	
				 // PID motor left
				currentTime_gl = HAL_GetTick();
				elapsedTime_gl = (double)(currentTime_gl - previousTime_gl)/1000.0; 
				count_gl = __HAL_TIM_GET_COUNTER(&htim3);
				if (count_gl>32767) count_gl=count_gl-65535;
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				anglel = (float)(count_gl) * 360/374.0/4.0;
				error_gl = angl - anglel;                                // determine error
        sumError_gl += error_gl * elapsedTime_gl;                // compute integral
        rateError_gl = (error_gl - lastError_gl)/elapsedTime_gl;   // compute derivative
        pwm_gl = kp_f*error_gl + ki_f*sumError_gl + kd_f*rateError_gl;                //PID output               
        lastError_gl = error_gl;	
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
				previousTime_gl = currentTime_gl;   
				if(pwm_gl >100)
				pwm_gl = 100; 
				if(pwm_gl <-100)
				pwm_gl = -100;
				if(pwm_gl>0)
				{
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);	
					TIM1->CCR3 = (int)pwm_gl;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);					
				}
				else
				 {					
//					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,fabs(pwm));	
					 TIM1->CCR3 = (int)(-pwm_gl);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);	
				 }			
				 HAL_Delay(49);
				 angr=set_angler;
				 angl=set_anglel;
				 
	}

	// vong vat can
int vong_vat_can()
	{
		e2=0;
		e3=0;
		error_gl=0,sumError_gl=0,rateError_gl=0,lastError_gl=0,anglel=0,count_gl=0;
		error_gr=0,sumError_gr=0,rateError_gr=0,lastError_gr=0,angler=0,count_gr=0;
		angl=0.0, angr=0.0;
		for(int i=0; i<20;i++){
		rotate(140,-140);
		print_data();
		}
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		angl=0.0, angr=0.0;
		error_gl=0,sumError_gl=0,rateError_gl=0,lastError_gl=0,anglel=0,count_gl=0;
		error_gr=0,sumError_gr=0,rateError_gr=0,lastError_gr=0,angler=0,count_gr=0;
		for(int j=0; j<20;j++){		
		forward(800,800);
		print_data();
		}
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		angl=0.0, angr=0.0;
		error_gl=0,sumError_gl=0,rateError_gl=0,lastError_gl=0,anglel=0,count_gl=0;
		error_gr=0,sumError_gr=0,rateError_gr=0,lastError_gr=0,angler=0,count_gr=0;
		for(int k=0; k<20;k++){		
		rotate(-245,245);
		print_data();
		}
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		for (int i=0; i<10; i++){
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;				
			motor(0,0);
			print_data();
			}			
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;				
		while (1){
		trig_hc04();
		read_hc04();	
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;				
			if (u16_ADCSensor[5]<500){
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0,pwm_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0,pwm_l=0;				
			motor(50,50);
			print_data();
				state=0;
		} 
		else {	
			angl=0.0, angr=0.0;
		error_gl=0,sumError_gl=0,rateError_gl=0,lastError_gl=0,anglel=0,count_gl=0;
		error_gr=0,sumError_gr=0,rateError_gr=0,lastError_gr=0,angler=0,count_gr=0;
			for(int i=0; i<20;i++){
		rotate(150,-150);
		print_data();
		}
			state=1;
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;		
			return 40;
		}
		}
	}
void tracking_line(){			
				v_r = sqrt(e2*e2 + e1*e1)/T_s;
				w_r = atan2(e2, e1)/T_s;
				v_c = v_r*cos(e3) + k_x * e1;
				w_c = w_r + v_r * (k_y * e2 + k_theta * sin(e3))*k;				 
				if (fabs(w_c)> w_max){ w_c = w_max*signnum_typical(w_c);} 
				if (fabs(v_c)> v_max){ v_c = v_max*signnum_typical(v_c);}  
				set_vwl=((1/r)*(v_c-(b/2)*w_c))*(60/(2*pi));
				set_vwr=((1/r)*(v_c+(b/2)*w_c))*(60/(2*pi));
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)u16_ADCSensor,7);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);	
	HAL_Delay(12000);
	int x=0,y=0;
	for (int i =0; i<20; i++){
	motor(50,50);
	}
	sprintf(TxBuff, "%s", "CLEARDATA\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%s", "LABEL, Time, Started Time, Date, e2, e3,");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
	sprintf(TxBuff, "%s", "RESETTIMER ");
	HAL_UART_Transmit(&huart1, (uint8_t *)TxBuff, strlen(TxBuff),1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// dem thoi gian lay mau
		currentTime= HAL_GetTick();
		elapsedTime = (double)(currentTime - previousTime);	
		if (elapsedTime > 50){
// doc cam bien do line
		trig_hc04();
		read_hc04();
// kiem tra vat can
		if (dis<25){
				vong_vat_can();	
//				if (dis <20){
//				error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
//				error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;	
////				for(int i=0; i<10; i++){
////				error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
////				error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;	
////				motor(-50,-50);					
////				error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
////				error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;	
////				}
//				}
				error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
				error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;	
				set_vwl=0;
			  set_vwr=0;
				}	
		else{ 
		if (state ==1){
		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;	
		state=0;
		}
// doc cam bien do line
		nume2 = (3*(u16_ADCSensor[6]-u16_ADCSensor[0])+2*(u16_ADCSensor[5]-u16_ADCSensor[1])+(u16_ADCSensor[4]-u16_ADCSensor[2]));
		dene2 = u16_ADCSensor[5]+u16_ADCSensor[4]+u16_ADCSensor[3]+u16_ADCSensor[2]+u16_ADCSensor[1]+u16_ADCSensor[6]+u16_ADCSensor[0];
//			nume2 = (2*(u16_ADCSensor[5]-u16_ADCSensor[1])+(u16_ADCSensor[4]-u16_ADCSensor[2]));
//		dene2 = u16_ADCSensor[5]+u16_ADCSensor[4]+u16_ADCSensor[3]+u16_ADCSensor[2]+u16_ADCSensor[1];
		if (u16_ADCSensor[5]>500 && u16_ADCSensor[1]>500) {
			nume2 = (3*(u16_ADCSensor[6]-200)+2*(u16_ADCSensor[5]-200)+(u16_ADCSensor[4]-200));
		dene2 = u16_ADCSensor[5]+u16_ADCSensor[4]+u16_ADCSensor[3]+u16_ADCSensor[2]+200+200+200;
//	e2=0.04;	
		}		
		e2 = 0.0155*(nume2/dene2);

		nume3=e2pre-e2;
		e3=atan(nume3/v_c*T_s);
// phat hien nga 3
//		if (u16_ADCSensor[6]>500 && u16_ADCSensor[0]>500) {
//	e3=pi/3;	
//		}
		e2pre=e2;				
		tracking_line();
		}
		print_data();
		previousTime=currentTime;
//		error_r=0,sumError_r=0,rateError_r=0,lastError_r=0;		
//		error_l=0,sumError_l=0,rateError_l=0,lastError_l=0;		
}
// phat hien diem dung
		if (u16_ADCSensor[3]<500 && u16_ADCSensor[1] <500 && u16_ADCSensor[2]<500 && u16_ADCSensor[4]<500 && u16_ADCSensor[5]<500 &&u16_ADCSensor[0]<500 && u16_ADCSensor[6]<500){
		set_vwr=0;
		set_vwl=0;		
		}
// dieu khien van toc 2 dong co
		motor(set_vwr,set_vwl);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  htim4.Init.Prescaler = 71;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
