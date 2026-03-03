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
#include <stdbool.h>
#include "wifi.h"

/* --- HTML Front-end (auto-refreshes every 2 seconds) --- */
const char* html_safe =
		"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=us-ascii\r\nConnection: close\r\n\r\n"
		    "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
		    "<meta http-equiv=\"refresh\" content=\"2\">"
		    "<style>"
		    "body{margin:0;height:100vh;display:flex;align-items:center;justify-content:center;background-color:#f4f7f6;font-family:system-ui,-apple-system,sans-serif;}"
		    ".card{background:#fff;padding:50px 30px;border-radius:24px;box-shadow:0 10px 30px rgba(0,0,0,0.08);text-align:center;max-width:90%;width:350px;}"
		    ".title{color:#2e7d32;margin:15px 0 10px 0;font-size:34px;font-weight:800;letter-spacing:-1px;}"
		    ".desc{color:#666;font-size:18px;margin:0;}"
		    ".footer{color:#aaa;font-size:13px;margin-top:30px;}"
		    ".icon-safe{font-size:50px;color:#2e7d32;font-weight:900;margin:0;line-height:1;letter-spacing:2px;}"
		    "</style></head><body>"
		    "<div class=\"card\">"
		    "<div class=\"icon-safe\">[ OK ]</div>"
		    "<h1 class=\"title\">Status: SAFE</h1>"
		    "<p class=\"desc\">Monitoring patient: <strong style=\"color:#333;\">Hongyu</strong></p>"
		    "<div class=\"footer\">System Healthy | Auto-updating (2s)</div>"
		    "</div></body></html>\r\n";

const char* html_alarm =
		"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=us-ascii\r\nConnection: close\r\n\r\n"
		    "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
		    "<meta http-equiv=\"refresh\" content=\"2\">"
		    "<style>"
		    "body{margin:0;height:100vh;display:flex;align-items:center;justify-content:center;background-color:#ffebee;font-family:system-ui,-apple-system,sans-serif;}"
		    ".card{background:#fff;padding:50px 30px;border-radius:24px;box-shadow:0 15px 35px rgba(211,47,47,0.2);text-align:center;max-width:90%;width:350px;border:3px solid #ffcdd2;}"
		    "@keyframes pulse{0%{transform:scale(1);}50%{transform:scale(1.15);}100%{transform:scale(1);}}"
		    ".icon-alarm{font-size:70px;color:#d32f2f;font-weight:900;margin:0;line-height:1;animation:pulse 1s infinite;}"
		    ".title{color:#d32f2f;margin:15px 0 10px 0;font-size:36px;font-weight:900;letter-spacing:-1px;}"
		    ".desc{color:#555;font-size:18px;margin:0 0 25px 0;}"
		    ".action{background:#fff5f5;color:#d32f2f;padding:15px;border-radius:12px;font-weight:700;font-size:16px;border:2px dashed #ef9a9a;}"
		    "</style></head><body>"
		    "<div class=\"card\">"
		    "<div class=\"icon-alarm\">!!!</div>"
		    "<h1 class=\"title\">FALL DETECTED</h1>"
		    "<p class=\"desc\">Patient <strong style=\"color:#000;\">Hongyu</strong> needs help!</p>"
		    "<div class=\"action\">Press the BLUE BUTTON<br>on the board to reset</div>"
		    "</div></body></html>\r\n";

static void UART1_Init(void);
static void SystemClock_Config(void); /* Required to boost clock to 80MHz for Wi-Fi SPI */

extern void initialise_monitor_handles(void);
extern int mov_avg(int N, int* accel_buff);
int mov_avg_C(int N, int* accel_buff);

UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi; /* Wi-Fi low-level SPI handle */

/* Bypasses linker errors since we are running bare-metal without FreeRTOS */
int RTOS_CREATE_SEM_MUTEX(void) { return 0; }
int RTOS_FREE_SEM_MUTEX(void) { return 0; }

// Hardware Initialization
void Buzzer_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int buzzer_on = 0;
void Buzzer_On(void) { buzzer_on = 1; }
void Buzzer_Off(void) {
    buzzer_on = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

// Reset Button initialization for wrong detection
void B2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int IsB2Pressed(void) {
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
}

void LED_Init(void) { BSP_LED_Init(LED2); }

// Main
int main(void)
{
    const int N = 4;
    HAL_Init();
    SystemClock_Config(); // Boost clock to 80MHz for stable Wi-Fi operations

    UART1_Init();
    Buzzer_Init();
    B2_Init();
    LED_Init();

    // Peripheral initializations
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_PSENSOR_Init();

    // Initialize variables to read data
    int accel_buff_x[4] = {0};
    int accel_buff_y[4] = {0};
    int accel_buff_z[4] = {0};
    int i = 0;
    char buffer[100];

    static int low_g_count = 0;
    static int low_g_timeout = 0;
    static float baseline_pressure = 0.0f;

    // Core FSM state flag for fall detection logic
    bool fall_detected = false;
    uint8_t IP_Addr[4];

    // Initialize Wi-Fi and start the TCP server before entering the main loop
    sprintf(buffer, "Initializing Wi-Fi...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    if (WIFI_Init() == WIFI_STATUS_OK) {
        if (WIFI_Connect("Hongyu", "12345678", WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) { // Key in your own hotspot and password
            WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)); // Get IP address assigned to the board
            sprintf(buffer, "Wi-Fi Ready! Dashboard IP: %d.%d.%d.%d.\r\n", IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            WIFI_StartServer(0, WIFI_TCP_PROTOCOL, 1, "", 80); // Open port 80
        }
    }

    while (1)
    {
        // 1. Accelerometer Processing
        int16_t accel_data_i16[3] = { 0 };
        BSP_ACCELERO_AccGetXYZ(accel_data_i16);

        accel_buff_x[i % 4] = accel_data_i16[0];
        accel_buff_y[i % 4] = accel_data_i16[1];
        accel_buff_z[i % 4] = accel_data_i16[2];

        // Filter the data of accelerometer
        float accel_filt_asm[3] = {0};
        accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8/1000.0f);
        accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8/1000.0f);
        accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8/1000.0f);

        float accel_sq = (accel_filt_asm[0] * accel_filt_asm[0]) +
                         (accel_filt_asm[1] * accel_filt_asm[1]) +
                         (accel_filt_asm[2] * accel_filt_asm[2]);

        // 2. Gyroscope Processing
        float gyro_data[3] = {0.0};
        BSP_GYRO_GetXYZ(gyro_data);

        float gyro_vel[3] = {0.0};
        gyro_vel[0] = (gyro_data[0] * 9.8 / 1000.0f);
        gyro_vel[1] = (gyro_data[1] * 9.8 / 1000.0f);
        gyro_vel[2] = (gyro_data[2] * 9.8 / 1000.0f);

        float gyro_sq = (gyro_vel[0] * gyro_vel[0]) + (gyro_vel[1] * gyro_vel[1]) + (gyro_vel[2] * gyro_vel[2]);

        float current_pressure = BSP_PSENSOR_ReadPressure();
        if (low_g_count == 0) {
            // Update the baseline pressure when falling is not detected
            baseline_pressure = (0.9f * baseline_pressure) + (0.1f * current_pressure);
        }

        // 3. Fall Detection Logic
        if (accel_sq < 15.0f) {
            low_g_count++;
            low_g_timeout = 0; // Reset timeout because we just found a low-G moment
        } else {
            low_g_timeout++;
            // If we've had 20 normal loops without a low-G moment, reset the whole thing
            if (low_g_timeout > 20) {
                low_g_count = 0;
            }
        }

        // Update the difference of pressure to estimate the altitude change
        float dP = current_pressure - baseline_pressure;

        // Check if both sustained low-G (weightlessness) and high rotation occurred
        if (low_g_count >= 3 && (gyro_sq > 500000.0f || dP > 0.15f)) {
            low_g_count = 0;

            // Update FSM flag upon fall detection
            fall_detected = true;

            sprintf(buffer, "!!! FALL DETECTED!!!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            Buzzer_On(); // Turn on the buzzer
        }

        // 4. Hardware UI / Button Controls
        if (IsB2Pressed()) {
            Buzzer_Off();

            // Clear the web alarm status immediately when the blue button is pressed
            if (fall_detected) {
                fall_detected = false;
                sprintf(buffer, "Alarm Cleared via Board Button.\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            }
        }

        // Alarm status. The LED blinks faster and the buzzer goes off
        if (buzzer_on) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
            BSP_LED_Toggle(LED2);
            for (volatile int j = 0; j < 500; j++);
        } else { // Normal status
            static int tick = 0;
            if (++tick >= 10) {
                BSP_LED_Toggle(LED2);
                tick = 0;
            }
        }

        // 5. Non-blocking Web Server Polling
        // Waits a maximum of 20ms for incoming requests to preserve sensor sampling frequency
        uint8_t RemoteIP[4];
        uint16_t RemotePort;
        // If wifi is connected
        if (WIFI_WaitServerConnection(0, 20, RemoteIP, sizeof(RemoteIP), &RemotePort) == WIFI_STATUS_OK) { // Wait 20 milliseconds for wifi connection
            uint8_t req_buf[256]; // Buffer to store data
            uint16_t req_len = 0, sent_len = 0;

            // If receive some data
            if (WIFI_ReceiveData(0, req_buf, sizeof(req_buf)-1, &req_len, 500) == WIFI_STATUS_OK && req_len > 0) {
                // Dispatch the corresponding HTML page based on the current FSM state
                if (fall_detected) {
                    WIFI_SendData(0, (uint8_t*)html_alarm, strlen(html_alarm), &sent_len, 1000); // Send the alarm web page
                } else {
                    WIFI_SendData(0, (uint8_t*)html_safe, strlen(html_safe), &sent_len, 1000); // Send the safe web page
                }
            }
            WIFI_CloseServerConnection(0); // Close connection
        }

        i++;
        // 80ms delay + 20ms Wi-Fi polling = perfectly maintains the 100ms (10Hz) sampling period
        HAL_Delay(80);
    }
}

static void UART1_Init(void)
{
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

// System clock configuration required for Wi-Fi SPI operations
static void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

// Wi-Fi SPI interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_1) {
    extern void SPI_WIFI_ISR(void);
    SPI_WIFI_ISR();
  }
}

void SPI3_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi); }

// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
