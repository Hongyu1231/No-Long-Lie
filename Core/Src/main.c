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

void Buzzer_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable Port A for PA4

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int buzzer_on = 0;

void Buzzer_On(void)
{
   buzzer_on = 1;   // 高电平 = 响
    // 如果你的蜂鸣器是低电平触发，就改成：
    // HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/*--------------------------- 蜂鸣器关 ---------------------------------------*/
void Buzzer_Off(void)
{
    buzzer_on = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}


void B2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // 内部上拉，按下时读到0
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int IsB2Pressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
}


int main(void)
{
    const int N = 4;
    HAL_Init();
    UART1_Init();
    Buzzer_Init();
    B2_Init();

    /* Peripheral initializations */
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();     // Initialize the Gyroscope

    int accel_buff_x[4] = {0};
    int accel_buff_y[4] = {0};
    int accel_buff_z[4] = {0};
    int i = 0;
    char buffer[100];

    static int low_g_count = 0;

    while (1)
    {
        // --- 1. Accelerometer Processing ---
        int16_t accel_data_i16[3] = { 0 };
        BSP_ACCELERO_AccGetXYZ(accel_data_i16);

        accel_buff_x[i % 4] = accel_data_i16[0];
        accel_buff_y[i % 4] = accel_data_i16[1];
        accel_buff_z[i % 4] = accel_data_i16[2];

        float accel_filt_asm[3] = {0};
        accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8/1000.0f);
        accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8/1000.0f);
        accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8/1000.0f);

        float accel_sq = (accel_filt_asm[0] * accel_filt_asm[0]) +
                         (accel_filt_asm[1] * accel_filt_asm[1]) +
                         (accel_filt_asm[2] * accel_filt_asm[2]);

        // --- 2. Gyroscope Processing ---
        float gyro_data[3] = {0.0};
        BSP_GYRO_GetXYZ(gyro_data);

        float gyro_vel[3] = {0.0};
        gyro_vel[0] = (gyro_data[0] * 9.8 / 1000.0f);
        gyro_vel[1] = (gyro_data[1] * 9.8 / 1000.0f);
        gyro_vel[2] = (gyro_data[2] * 9.8 / 1000.0f);

        float gyro_sq = (gyro_vel[0] * gyro_vel[0]) + (gyro_vel[1] * gyro_vel[1]) + (gyro_vel[2] * gyro_vel[2]);

        // 3. Print values for monitoring
        sprintf(buffer, "ACCEL_SQ: %.2f | GYRO_SQ: %.2f\r\n", accel_sq, gyro_sq);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

        // --- 4. Fall Detection Logic ---
        if (accel_sq < 15.0f) {
            low_g_count++;
        } else {
            low_g_count = 0;
        }

        // Check if both sustained low-G (weightlessness) and high rotation occurred
        if (low_g_count >= 3 && gyro_sq > 500000.0f) {
            low_g_count = 0;
            sprintf(buffer, "!!! FALL DETECTED (G-Drop + Tumble) !!!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            Buzzer_On();
        }

        // Buzzer/Button Controls
        if (IsB2Pressed()) {
            Buzzer_Off();
        }

        if (buzzer_on) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
            for (volatile int j = 0; j < 500; j++);
        }

        i++;
        HAL_Delay(100);
    }
}


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
