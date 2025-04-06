/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
#include "lsm6dsl.h"
#include "hts221.h"
#include "lps22hb.h"
#include "lis3mdl.h"

#include "b_l475e_iot01a1_bus.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_hal.h"

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
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
LSM6DSL_Object_t MotionSensor;
HTS221_Object_t TempHumSensor;
LPS22HB_Object_t PressSensor;
LIS3MDL_Object_t MagnetoSensor;

// Variables for sensor data
float temperature;
float humidity;
float pressure;
LSM6DSL_Axes_t gyro_axes;
LIS3MDL_Axes_t mag_axes;

volatile uint32_t dataRdyIntReceived;

/* AI variables*/
ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char* activities[AI_NETWORK_OUT_1_SIZE] = {
  "stationary", "walking", "running"
};
ai_buffer * ai_input;
ai_buffer * ai_output;
/* USER CODE END PV */



/* --------------------------------------------------------------- */
/* FIFO and scheduling definitions */
/* --------------------------------------------------------------- */

#define FIFO_SIZE 10

typedef enum {
    SENSOR_TEMP = 0,
    SENSOR_HUMIDITY,
    SENSOR_PRESSURE,
    SENSOR_ACCEL,
    SENSOR_GYRO,
    SENSOR_MAGNETO,
    SENSOR_COUNT
} SensorType;

const char *SENSOR_NAMES[SENSOR_COUNT] = {
    "Temperature",
    "Humidity",
    "Pressure",
    "Acceleration",
    "Gyroscope",
    "Magneto"
};

const char *SENSOR_UNITS[SENSOR_COUNT] = {
    "degreeC",
    "%rH",
    "kPa",
    "g",
    "dps",
    "G"
};

typedef struct {
    uint32_t timestamp;   // using HAL_GetTick() for timestamp (ms)
    float value;
} SensorFIFOEntry;

typedef struct {
    SensorFIFOEntry buffer[FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} FIFOBuffer;

/* Global FIFO buffers for each sensor */
FIFOBuffer fifo[SENSOR_COUNT];

/* Polling intervals (in ms) for normal and low power modes */
const uint32_t NORMAL_MODE_INTERVAL[SENSOR_COUNT] = {500, 500, 500, 500, 500, 500};
const uint32_t LP_MODE_INTERVAL[SENSOR_COUNT] = {5000, 5000, 5000, 5000, 5000, 5000};

int POLL_INTERVAL[SENSOR_COUNT] = {NORMAL_MODE_INTERVAL[0],
                                   NORMAL_MODE_INTERVAL[1],
                                   NORMAL_MODE_INTERVAL[2],
                                   NORMAL_MODE_INTERVAL[3],
                                   NORMAL_MODE_INTERVAL[4],
                                   NORMAL_MODE_INTERVAL[5]};
int IsLowPowerMode = 0;
uint32_t lastPollTime[SENSOR_COUNT] = {0};

/* FIFO storage intervals (ms) for each sensor */
const int FIFO_INTERVAL[SENSOR_COUNT] = {1000, 1000, 1000, 100, 1000, 1000};
uint32_t lastFIFOTime[SENSOR_COUNT] = {0};
uint32_t startCycles[SENSOR_COUNT] = {0};
int readCount[SENSOR_COUNT] = {0};
int discardCount[SENSOR_COUNT] = {0};
int criticalCount[SENSOR_COUNT] = {0};
uint32_t lowpClock;

/* --------------------------------------------------------------- */
/* FIFO helper functions */
/* --------------------------------------------------------------- */
void FIFO_Init(FIFOBuffer *fifo) {
    fifo->head = fifo->tail = fifo->count = 0;
}

int FIFO_IsFull(FIFOBuffer *fifo) {
    return (fifo->count >= FIFO_SIZE);
}

int FIFO_IsEmpty(FIFOBuffer *fifo) {
    return (fifo->count == 0);
}

int FIFO_Enqueue(FIFOBuffer *fifo, SensorFIFOEntry entry) {
    if(FIFO_IsFull(fifo))
        return 0; // Buffer full
    fifo->buffer[fifo->head] = entry;
    fifo->head = (fifo->head + 1) % FIFO_SIZE;
    fifo->count++;
    return 1;
}

int FIFO_Dequeue(FIFOBuffer *fifo, SensorFIFOEntry *entry) {
    if(FIFO_IsEmpty(fifo))
        return 0; // Buffer empty
    *entry = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
    fifo->count--;
    return 1;
}

/* FIFO selection schemes */
typedef enum {
    SELECTION_RANDOM,
    SELECTION_FULL_BUFFER,
    SELECTION_PREDICTIVE
} SelectionScheme;

int SelectFIFO(SelectionScheme scheme) {
    int selected = -1;
    switch(scheme) {
        case SELECTION_RANDOM: {
            int available[SENSOR_COUNT];
            int count = 0;
            for(int i = 0; i < SENSOR_COUNT; i++){
                if(!FIFO_IsEmpty(&fifo[i])){
                    available[count++] = i;
                }
            }
            if(count > 0){
                selected = available[rand() % count];
            }
            break;
        }
        case SELECTION_FULL_BUFFER: {
            int maxCount = 0;
            for(int i = 0; i < SENSOR_COUNT; i++){
                if(fifo[i].count > maxCount){
                    maxCount = fifo[i].count;
                    selected = i;
                }
            }
            break;
        }
        case SELECTION_PREDICTIVE: {
            float maxRatio = 0.0f;
            for(int i = 0; i < SENSOR_COUNT; i++){
                uint32_t interval = NORMAL_MODE_INTERVAL[i]; // for simplicity, use normal intervals
                float ratio = (float)fifo[i].count / (float)interval;
                if(ratio > maxRatio && !FIFO_IsEmpty(&fifo[i])){
                    maxRatio = ratio;
                    selected = i;
                }
            }
            break;
        }
        default:
            break;
    }
    return selected;
}

void ProcessFIFOItem(int sensor) {
    SensorFIFOEntry entry;
    char msg[80];
    if(FIFO_Dequeue(&fifo[sensor], &entry)) {
        sprintf(msg, "FIFO Process: %s = %.2f %s at %lu ms\r\n",
                SENSOR_NAMES[sensor], entry.value, SENSOR_UNITS[sensor], entry.timestamp);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    else{
		sprintf(msg, "FIFO Full:  %s, drop data. \r\n", SENSOR_NAMES[sensor]);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
}



/* Global flag array indicating data ready for each sensor */
volatile uint8_t sensorReady[SENSOR_COUNT] = {0};

/* Interrupt callback: set the flag corresponding to the sensor when DRDY occurs */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == HTS221_DRDY_EXTI15_Pin) {
        /* HTS221 DRDY: set both temperature and humidity flags */
        sensorReady[SENSOR_TEMP] = 1;
        sensorReady[SENSOR_HUMIDITY] = 1;
    }
    else if(GPIO_Pin == LPS22HB_INT_DRDY_EXTI0_Pin) {
        sensorReady[SENSOR_PRESSURE] = 1;
        sensorReady[SENSOR_MAGNETO] = 1;
    }
    else if(GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin) {
        /* LSM6DSL DRDY: set flags for both acceleration and gyroscope */
        sensorReady[SENSOR_ACCEL] = 1;
        sensorReady[SENSOR_GYRO] = 1;
        dataRdyIntReceived++;
    }
    //else if(GPIO_Pin == LIS3MDL_DRDY_EXTI8_Pin) {
        //sensorReady[SENSOR_MAGNETO] = 1;
    //}
    else if(GPIO_Pin == BUTTON_EXTI13_Pin) {
    }

}

/* Sensor reading wrapper functions using driver API calls */
float Read_Temperature(void) {
    float temp = 0.0f;
    if (HTS221_TEMP_GetTemperature(&TempHumSensor, &temp) == HTS221_OK) {
        return temp;
    }
    return 0.0f;
}

float Read_Humidity(void) {
    float hum = 0.0f;
    if (HTS221_HUM_GetHumidity(&TempHumSensor, &hum) == HTS221_OK) {
        return hum;
    }
    return 0.0f;
}

float Read_Pressure(void) {
    float pres = 0.0f;
    if (LPS22HB_PRESS_GetPressure(&PressSensor, &pres) == LPS22HB_OK) {
        return pres;
    }
    return 0.0f;
}

float Read_Accel(void) {
    LSM6DSL_Axes_t acc;
    if (LSM6DSL_ACC_GetAxes(&MotionSensor, &acc) == LSM6DSL_OK) {
        // Normalize or convert as needed; here we compute the magnitude in "g"
        float ax = (float)acc.x / 4000.0f;
        float ay = (float)acc.y / 4000.0f;
        float az = (float)acc.z / 4000.0f;
        return sqrtf(ax * ax + ay * ay + az * az);
    }
    return 0.0f;
}

float Read_Gyro(void) {
    LSM6DSL_Axes_t gyro;
    if (LSM6DSL_GYRO_GetAxes(&MotionSensor, &gyro) == LSM6DSL_OK) {
        float gx = (float)gyro.x / 1000.0f;
        float gy = (float)gyro.y / 1000.0f;
        float gz = (float)gyro.z / 1000.0f;
        return sqrtf(gx * gx + gy * gy + gz * gz);
    }
    return 0.0f;
}

float Read_Magneto(void) {
    LIS3MDL_Axes_t mag;
    if (LIS3MDL_MAG_GetAxes(&MagnetoSensor, &mag) == LIS3MDL_OK) {
        float mx = (float)mag.x / 1000.0f;
        float my = (float)mag.y / 1000.0f;
        float mz = (float)mag.z / 1000.0f;
        return sqrtf(mx * mx + my * my + mz * mz);
    }
    return 0.0f;
}

float Read_Sensor(SensorType sensor) {
    switch(sensor) {
        case SENSOR_TEMP:      return Read_Temperature();
        case SENSOR_HUMIDITY:  return Read_Humidity();
        case SENSOR_PRESSURE:  return Read_Pressure();
        case SENSOR_ACCEL:     return Read_Accel();
        case SENSOR_GYRO:      return Read_Gyro();
        case SENSOR_MAGNETO:   return Read_Magneto();
        default:               return 0.0f;
    }
}




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float * values, uint32_t len);
/* USER CODE END PFP */



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	MX_DFSDM1_Init();
	MX_QUADSPI_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USB_DEVICE_Init();
	MX_GPIO_Init();

	dataRdyIntReceived = 0;
	MEMS_Init();
	AI_Init();
    uint32_t write_index = 0;


	/* Initialize FIFO buffers for each sensor */
	for (int i = 0; i < SENSOR_COUNT; i++) {
	    FIFO_Init(&fifo[i]);
	    lastPollTime[i] = HAL_GetTick();
	}

	/* Enable DWT cycle counter for timing measurements */
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	}
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	lowpClock = HAL_GetTick();

	/** Choose a FIFO selection scheme
    SELECTION_RANDOM,
    SELECTION_FULL_BUFFER,
    SELECTION_PREDICTIVE*/
	SelectionScheme currentScheme = SELECTION_FULL_BUFFER;
	char msg[80];
	sprintf(msg, "Init DONE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);


	while (1)
	{
		if (dataRdyIntReceived != 0) {
			dataRdyIntReceived = 0;
			LSM6DSL_Axes_t acc_axes;
			LSM6DSL_ACC_GetAxes(&MotionSensor, &acc_axes);
			/* Normalize data to [-1; 1] and accumulate into input buffer */
			/* Note: window overlapping can be managed here */
			aiInData[write_index + 0] = (float) acc_axes.x / 4000.0f;
			aiInData[write_index + 1] = (float) acc_axes.y / 4000.0f;
			aiInData[write_index + 2] = (float) acc_axes.z / 4000.0f;
			write_index += 3;

			if (write_index == AI_NETWORK_IN_1_SIZE) {
				write_index = 0;
				sprintf(msg, "Running inference\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
				AI_Run(aiInData, aiOutData);

				/* Output results */
				for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
					char msg[80];
					sprintf(msg, "%8.6f ",aiOutData[i]);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
				}
				uint32_t class = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
				sprintf(msg, ": %d - %s\r\n",(int) class, activities[class]);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

			}
		}


		uint32_t now = HAL_GetTick();
		/* Poll each sensor based on its own poll interval */
		for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
			if (now - lastPollTime[sensor] >= POLL_INTERVAL[sensor]) {
				lastPollTime[sensor] = now;
				float value = Read_Sensor((SensorType)sensor);

				SensorFIFOEntry entry;
				entry.timestamp = now;
				entry.value = value;
				FIFO_Enqueue(&fifo[sensor], entry);

				sprintf(msg, "FIFO Stored:  %s: %.2f(%s) at %lu ms\r\n",
				SENSOR_NAMES[sensor], value, SENSOR_UNITS[sensor], entry.timestamp);
				//HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
			}
		}

		/* Run cyclic scheduler: process one FIFO item from a selected sensor FIFO */
		int selected = SelectFIFO(currentScheme);
		if (selected >= 0) {
			ProcessFIFOItem(selected);
		}




	}//while end

}






/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MEMS_Init(void)
{
	char msg[80];

	/* ----------- LSM6DSL Initialization (Acceleration & Gyroscope) ------------- */

	LSM6DSL_IO_t io_ctx;
	uint8_t id;
	LSM6DSL_AxesRaw_t axes;

	/* Link I2C functions to the LSM6DSL driver */
	io_ctx.BusType   = LSM6DSL_I2C_BUS;
	io_ctx.Address   = LSM6DSL_I2C_ADD_L;    // Use low address if SA0 is low (e.g. 0xD5)
	io_ctx.Init      = BSP_I2C2_Init;
	io_ctx.DeInit    = BSP_I2C2_DeInit;
	io_ctx.ReadReg   = BSP_I2C2_ReadReg;
	io_ctx.WriteReg  = BSP_I2C2_WriteReg;
	io_ctx.GetTick   = BSP_GetTick;
	LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);

	/* Read WHO_AM_I register */
	LSM6DSL_ReadID(&MotionSensor, &id);
	if (id != LSM6DSL_ID) {
		Error_Handler();
	}

	/* Initialize the sensor */
	LSM6DSL_Init(&MotionSensor);

	/* Configure the accelerometer */
	LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 1.0f);  /* 26 Hz */
	LSM6DSL_ACC_SetFullScale(&MotionSensor, 4);           /* ±4g */
	LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);     /* Enable DRDY */
	LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes);         /* Clear DRDY */

	/* Start accelerometer & gyroscope */
	LSM6DSL_ACC_Enable(&MotionSensor);
	LSM6DSL_GYRO_Enable(&MotionSensor);

	sprintf(msg, "LSM6DSL INIT DONE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);


	/* ----------- HTS221 Initialization (Temperature & Humidity) ------------- */

	HTS221_IO_t hts_io;

	memset(&hts_io, 0, sizeof(hts_io));
	hts_io.BusType   = HTS221_I2C_BUS;
	hts_io.Address   = HTS221_I2C_ADDRESS;   // Defined in hts221_reg.h (typically 0xBF)
	hts_io.Init      = BSP_I2C2_Init;
	hts_io.DeInit    = BSP_I2C2_DeInit;
	hts_io.ReadReg   = BSP_I2C2_ReadReg;
	hts_io.WriteReg  = BSP_I2C2_WriteReg;
	hts_io.GetTick   = BSP_GetTick;
	hts_io.Delay     = HAL_Delay;
	HTS221_RegisterBusIO(&TempHumSensor, &hts_io);

	/* Initialize the sensor */
	HTS221_Init(&TempHumSensor);

	/* Set desired ODR for humidity and temperature channels */
	HTS221_HUM_SetOutputDataRate(&TempHumSensor, 12.5f);
	HTS221_TEMP_SetOutputDataRate(&TempHumSensor, 12.5f);
	HTS221_Enable_DRDY_Interrupt(&TempHumSensor);

	sprintf(msg, "HTS221 INIT DONE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);



	/* ----------- LPS22HB Initialization (Pressure) ------------- */

	LPS22HB_IO_t lps_ctx;
	uint8_t lps_id;

	memset(&lps_ctx, 0, sizeof(lps_ctx));
	lps_ctx.BusType   = LPS22HB_I2C_BUS;
	lps_ctx.Address   = LPS22HB_I2C_ADD_H;
	lps_ctx.Init      = BSP_I2C2_Init;
	lps_ctx.DeInit    = BSP_I2C2_DeInit;
	lps_ctx.ReadReg   = BSP_I2C2_ReadReg;
	lps_ctx.WriteReg  = BSP_I2C2_WriteReg;
	lps_ctx.GetTick   = BSP_GetTick;
	lps_ctx.Delay     = HAL_Delay;
	LPS22HB_RegisterBusIO(&PressSensor, &lps_ctx);

	/* Read WHO_AM_I register */
	if (LPS22HB_ReadID(&PressSensor, &lps_id) != LPS22HB_OK || lps_id != LPS22HB_ID) {
		Error_Handler();
	}
	/* Initialize the sensor */
	LPS22HB_Init(&PressSensor);

	/* Set the output data rate for pressure */
	LPS22HB_PRESS_SetOutputDataRate(&PressSensor, 25.0f);
	LPS22HB_TEMP_SetOutputDataRate(&PressSensor, 25.0f);

	LPS22HB_PRESS_Enable(&PressSensor);

	sprintf(msg, "LPS22HB INIT DONE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);


	/* ----------- LIS3MDL Initialization (Magnetometer) ------------- */

	LIS3MDL_IO_t lis3_ctx;
	uint8_t lis3_id;

	memset(&lis3_ctx, 0, sizeof(lis3_ctx));
	lis3_ctx.BusType   = LIS3MDL_I2C_BUS;
	/* For your board, if SA0 is high then use the high address */
	lis3_ctx.Address   = LIS3MDL_I2C_ADD_H;    // Typically 0x3D if SA0 is high
	lis3_ctx.Init      = BSP_I2C2_Init;
	lis3_ctx.DeInit    = BSP_I2C2_DeInit;
	lis3_ctx.ReadReg   = BSP_I2C2_ReadReg;
	lis3_ctx.WriteReg  = BSP_I2C2_WriteReg;
	lis3_ctx.GetTick   = BSP_GetTick;
	lis3_ctx.Delay     = HAL_Delay;
	LIS3MDL_RegisterBusIO(&MagnetoSensor, &lis3_ctx);

	if (LIS3MDL_ReadID(&MagnetoSensor, &lis3_id) != LIS3MDL_OK || lis3_id != LIS3MDL_ID) {
		Error_Handler();
	}

	LIS3MDL_Init(&MagnetoSensor);

	/* Configure the magnetometer: set ODR and full scale */

	LIS3MDL_MAG_SetOutputDataRate(&MagnetoSensor, 80.0f);
	LIS3MDL_MAG_SetFullScale(&MagnetoSensor, 4);
	        /* If an interrupt is required for DRDY, enable it by writing to CTRL_REG3 or similar.
	           For example:
	             uint8_t int_conf = DESIRED_VALUE; // Determine appropriate value from datasheet.
	             if (LIS3MDL_Write_Reg(&MagnetoSensor, LIS3MDL_CTRL_REG3, int_conf) != LIS3MDL_OK) {
	                 Error_Handler();
	             }
	        */
	LIS3MDL_MAG_Enable(&MagnetoSensor);

	sprintf(msg, "LIS3MDL INIT DONE\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

}


static void AI_Init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
}

static void AI_Run(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}



int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}

static uint32_t argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}
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
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_Delay(50); /* wait 50 ms */
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
