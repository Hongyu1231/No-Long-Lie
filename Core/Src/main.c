  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/


/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"

#include "stdio.h"
#include "string.h"
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;


int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();

	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();

	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	int i=0;
	int delay_ms=100; //change delay time to suit your code

	while (1)
	{

		BSP_LED_Toggle(LED2);		// This function helps to toggle the current LED state

		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings of accelerometer
		/********Function call to read accelerometer values*********/
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		//Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; //acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; //acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; //acceleration along Z-Axis


		// ********* Read gyroscope values *********/
		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		//The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));


		//Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3]={0}; // final value of filtered acceleration values

		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);


		//Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
//		float accel_filt_c[3]={0};
//
//		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
//		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
//		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		/***************************UART transmission*******************************************/
		char buffer[150]; // Create a buffer large enough to hold the text

		/******Transmitting results of C execution over UART*********/
//		if(i>=3)
//		{
//			// 1. First printf() Equivalent
//			sprintf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
//					accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			/******Transmitting results of asm execution over UART*********/

//			// 1. First printf() Equivalent
//			sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
//					accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			/******Transmitting Gyroscope readings over UART*********/

//			// 1. First printf() Equivalent
//			sprintf(buffer, "Gyroscope sensor readings:\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//			// 2. Second printf() (with Floats) Equivalent
//			// Note: Requires -u _printf_float to be enabled in Linker settings
//			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n\n",
//					gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//		}
//
		HAL_Delay(delay_ms);	// 1 second delay
//
		i++;

		// ********* Fall detection *********/
		// write your program from here:
		// 1. Calculate the squared magnitude of the acceleration (using filtered asm results)
		float accel_sq = (accel_filt_asm[0] * accel_filt_asm[0]) + (accel_filt_asm[1] * accel_filt_asm[1]) + (accel_filt_asm[2] * accel_filt_asm[2]);

		// Adjust the threshold
//		sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//		sprintf(buffer, "accel_sq： %f\r\n", accel_sq);
//		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		// 2. Calculate squared magnitude of Gyroscope (angular velocity)
	    float gyro_sq = (gyro_velocity[0] * gyro_velocity[0]) + (gyro_velocity[1] * gyro_velocity[1]) + (gyro_velocity[2] * gyro_velocity[2]);

	    // Adjust the threshold
//	    sprintf(buffer, "Gyroscope sensor readings:\r\n");
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//	    sprintf(buffer, "gyro_sq： %f\r\n", gyro_sq);
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		// 3. Get Barometer data
		float current_pressure = BSP_PSENSOR_ReadPressure();

	    // Adjust the threshold
//	    sprintf(buffer, "Barometer sensor readings:\r\n");
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//	    sprintf(buffer, "current_pressure: %f\r\n", current_pressure);
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		// 4. State machine variables: 'static' retains their values across while-loop iterations
		static uint8_t fall_state = 0;
		static uint8_t timeout_timer = 0;
		static uint8_t rotation_detected = 0;
		static float baseline_pressure = 0.0f; // The pressure before falling down

		// Physical characteristics thresholds for a fall (compared using squared values):
		float ACCEL_FREEFALL_SQ = 20.0f; // Normal is 90
		float GYRO_TUMBLE_SQ = 500000.0f; // Normal is 10**3
		float PRESSURE_DROP_THRESHOLD = 0.09f;
		float ACCEL_CATASTROPHIC_SQ = 400.0f;
		float ACCEL_IMPACT_LOWER_SQ = 110.0f;
		float ACCEL_IMPACT_HIGHER_SQ = 170.0f;

		// fall detection logic
		// Car crash or extreme violence detection
		if (accel_sq > ACCEL_CATASTROPHIC_SQ)
		{
		    delay_ms = 50;
		    sprintf(buffer, "!!! CATASTROPHIC IMPACT DETECTED !!! Acc:%.1f\r\n", accel_sq);
		    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		    fall_state = 0; // Reset normal state machine
		}

		// NORMAL FALL LOGIC (Syncopal fall or Trip)
		else if (accel_sq < ACCEL_FREEFALL_SQ) // Free-fall detected
		{

			// Debug
		    sprintf(buffer, "Free-fall detected!!! accel_sq: %f\r\n", accel_sq);
		    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			if (fall_state == 0)
			{
			    // Set the pressure baseline when free-fall just starts
			    baseline_pressure = current_pressure;
			}
			fall_state = 1; // Phase 1 : free-fall detected
			timeout_timer = 0; // Reset Timeout-Timer
			rotation_detected = 0;
		}

		else if (fall_state == 1)
			{
				timeout_timer++;

				// Lock the flag if a rapid tumble is detected at any point during the fall
				if (gyro_sq > GYRO_TUMBLE_SQ)
				{
					rotation_detected = 1;
					sprintf(buffer, "Rotation detected!!! gyro_sq: %f\r\n", gyro_sq);
					HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				}

				if (accel_sq > ACCEL_IMPACT_LOWER_SQ && accel_sq < ACCEL_IMPACT_HIGHER_SQ)
				{
					// Phase 2: High impact detected
					float pressure_diff = current_pressure - baseline_pressure;

					// Debug
				    sprintf(buffer, "Impact detected!!! accel_sq: %f\r\n", accel_sq);
				    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

					// Check for rotation OR altitude drop
					if ((rotation_detected == 1 || gyro_sq > GYRO_TUMBLE_SQ) || (pressure_diff > PRESSURE_DROP_THRESHOLD))
					{
						delay_ms = 50;
						sprintf(buffer, "!!! TRUE FALL !!! Acc:%.1f, dP:%.3f\r\n", accel_sq, pressure_diff);
						HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
					}
					fall_state = 0; // Reset state machine
				}

				else if (timeout_timer > 30)
				{
					// Debug
				    sprintf(buffer, "Timeout detected!!!\r\n");
				    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
					// Timeout: Free-fall without subsequent impact (e.g., just lowering the board)
					fall_state = 0; // Reset state
				}
			}

		else
		{
			// Normal state: Change delay_ms to 100ms
			delay_ms = 100;
		}
	}
}



//int mov_avg_C(int N, int* accel_buff)
//{ 	// The implementation below is inefficient and meant only for verifying your results.
//	int result=0;
//	for(int i=0; i<N;i++)
//	{
//		result+=accel_buff[i];
//	}
//
//	result=result/4;
//
//	return result;
//}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
