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
// === Wi-Fi Headers and Global Object ===
#include "../../Drivers/BSP/Components/es_wifi/es_wifi.h"

// The core Wi-Fi object that handles all network operations
ES_WIFIObject_t EsWifiObj;

#include "stdio.h"
#include "string.h"
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;

// ==================== WI-FI Low-Level SPI Bridge Code ====================
SPI_HandleTypeDef hspi3;

// Microsecond delay function required by the Wi-Fi chip core driver
void SPI_WIFI_Delay(uint32_t Delay) {
    HAL_Delay(Delay);
}

// 1. Initialize SPI3 and all related control pins
int8_t SPI_WIFI_Init(uint16_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks for all related GPIO ports and SPI3 */
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Configure SPI3 data pins: PC10 (SCK), PC11 (MISO), PC12 (MOSI) */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure CS (Chip Select) pin: PE0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET); // Default to high (deselected)

    /* Configure Reset pin: PE8 and Wakeup pin: PB13 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); // Default to reset state

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Default to low

    /* Configure Data Ready interrupt pin: PE1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // Enable EXTI1 interrupt (corresponds to PE1)
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* Initialize SPI3 hardware parameters */
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_16BIT; // Inventek Wi-Fi module typically uses 16-bit transmission
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        return -1; // Initialization failed
    }
    return 0; // Success
}

// 2. Receive data wrapper function
int8_t SPI_WIFI_ReceiveData(uint8_t *pData, uint16_t len, uint32_t timeout) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // Pull CS low to select the chip
    int8_t status = (HAL_SPI_Receive(&hspi3, pData, len/2, timeout) == HAL_OK) ? 0 : -1;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);   // Pull CS high to deselect
    return status;
}

// 3. Send data wrapper function
int8_t SPI_WIFI_SendData(uint8_t *pData, uint16_t len, uint32_t timeout) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // Pull CS low
    int8_t status = (HAL_SPI_Transmit(&hspi3, pData, len/2, timeout) == HAL_OK) ? 0 : -1;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);   // Pull CS high
    return status;
}

// 4. Wrapper function to read Data Ready (PE1) pin state
uint8_t SPI_WIFI_DataReady(void) {
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET) ? 1 : 0;
}
// ==================== End of WI-FI Low-Level Bridge Code ====================

void Buzzer_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable Port A for PA4

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Buzzer_Test(void) {
    // OUTER LOOP: This counts the number of beeps
    for(int beep = 0; beep < 3; beep++) {

        // INNER LOOP: This creates the vibration (the sound)
        for(int i = 0; i < 500; i++) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
            HAL_Delay(1);
        }

        // 1. FORCE SILENCE: Turn the pin off so it doesn't stay "High"
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

        // 2. GAP: Wait for half a second before the next beep starts
        HAL_Delay(500);
    }
}

int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	Buzzer_Init();

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

// ==================== WI-FI BOOT & CONNECT ====================
char buffer[150]; // Move buffer declaration here
sprintf(buffer, "\r\n--- Initializing Wi-Fi Module ---\r\n");
HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

// Bind custom low-level SPI functions to the core driver object
extern ES_WIFIObject_t EsWifiObj;

// Boot up the Wi-Fi chip
if (ES_WIFI_Init(&EsWifiObj) == ES_WIFI_STATUS_OK)
{
    sprintf(buffer, "Wi-Fi Chip Booted! Connecting to Router...\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	if (ES_WIFI_Connect(&EsWifiObj, "Hongyu", "12345678", ES_WIFI_SEC_WPA_WPA2) == ES_WIFI_STATUS_OK)
	{
	    ES_WIFI_GetNetworkSettings(&EsWifiObj);
	    sprintf(buffer, "Wi-Fi Connected! My IP is %d.%d.%d.%d\r\n\n",
	    		EsWifiObj.NetSettings.IP_Addr[0], EsWifiObj.NetSettings.IP_Addr[1],
				EsWifiObj.NetSettings.IP_Addr[2], EsWifiObj.NetSettings.IP_Addr[3]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	}
	else
	{
	    sprintf(buffer, "Wi-Fi Connection Failed!\r\n\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	}
}
// ==============================================================

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

		// Adjust here to modify the fall detection
		// Physical characteristics thresholds for a fall (compared using squared values):
		float ACCEL_FREEFALL_SQ = 20.0f; // Normal is 90
		float GYRO_TUMBLE_SQ = 500000.0f; // Normal is 10**3
		float PRESSURE_DROP_THRESHOLD = 0.1f;
		float ACCEL_CATASTROPHIC_SQ = 500.0f;
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

				// Phase 2: High impact detected
				if (accel_sq > ACCEL_IMPACT_LOWER_SQ && accel_sq < ACCEL_IMPACT_HIGHER_SQ)
				{
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
						Buzzer_Test();

						// Define the connection configuration object
						ES_WIFI_Conn_t udp_conn;

					    // [IMPORTANT!] Change to the IP address assigned to your computer
						udp_conn.RemoteIP[0] = 192;
						udp_conn.RemoteIP[1] = 168;
						udp_conn.RemoteIP[2] = 43;
					    udp_conn.RemoteIP[3] = 189;

					    udp_conn.LocalPort = 8080;
					    udp_conn.RemotePort = 8080;
					    udp_conn.Type = ES_WIFI_UDP_CONNECTION;
					    udp_conn.Number = 0; // Use socket 0

					    uint8_t alert_msg[] = "!!! ALERT: BRUCE HAS FALLEN !!!\r\n";
					    uint16_t sent_len = 0;

					    // Start connection, send data, and close
					    if(ES_WIFI_StartClientConnection(&EsWifiObj, &udp_conn) == ES_WIFI_STATUS_OK)
					    {
					        ES_WIFI_SendData(&EsWifiObj, udp_conn.Number, alert_msg, sizeof(alert_msg)-1, &sent_len, 2000);
					        ES_WIFI_StopClientConnection(&EsWifiObj, &udp_conn);
					    }
					}

					fall_state = 0; // Reset state machine
				}

				// Timeout: Free-fall without subsequent impact (e.g., just lowering the board)
				else if (timeout_timer > 30)
				{
					// Debug
				    sprintf(buffer, "Timeout detected!!!\r\n");
				    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
					fall_state = 0; // Reset state
				}
			}

		// Normal state: Change delay_ms to 100ms
		else
		{
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


// External declaration from es_wifi_conf.h
extern void SPI_WIFI_ISR(void);

void SPI_WIFI_ISR(void)
{

}

// Triggered when PE1 (Data Ready) is pulled high by the Wi-Fi module
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1)
    {
        // Call the core driver's Interrupt Service Routine (ISR)
        SPI_WIFI_ISR();
    }
}
