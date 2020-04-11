/* GPS NMEA Recorder

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "ili9340.h"
#include "fontx.h"

static const char *TAG = "NMEA";

static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;

//#define TXD_GPIO	0
// You have to set these CONFIG value using menuconfig.
#if 0
#define CONFIG_UART_RXD_GPIO	22
#define CONFIG_UART_TXD_GPIO	21
#endif

#define SCREEN_WIDTH	320
#define SCREEN_HEIGHT	240
#define CS_GPIO			14
#define DC_GPIO			27
#define RESET_GPIO		33
#define BL_GPIO			32
#define DISPLAY_LENGTH	26
#define GPIO_INPUT_A	GPIO_NUM_39
#define GPIO_INPUT_B	GPIO_NUM_38
#define GPIO_INPUT_C	GPIO_NUM_37

#define MAX_PAYLOAD		256
typedef struct {
	uint16_t command;
	size_t	 length;
	uint8_t  payload[MAX_PAYLOAD];
	TaskHandle_t taskHandle;
} CMD_t;

#define CMD_START_RECORD			100
#define CMD_STOP_RECORD				150
#define CMD_START_PLAYBACK_ONCE		200
#define CMD_START_PLAYBACK_REPEAT	250
#define CMD_STOP_PLAYBACK			300
#define CMD_NMEA					400
#define CMD_TIMER					500

typedef struct {
	char		fname[64];
	FILE*		f;
	uint16_t		mode;
	uint16_t	record;
} LOG_t;

typedef struct {
	uint8_t	_time[20];
	uint8_t _valid[10];
	uint8_t _lat1[20];
	uint8_t _lat2[10];
	uint8_t _lon1[20];
	uint8_t _lon2[10];
	uint8_t _speed[10];
	uint8_t _orient[10];
	uint8_t	_date[20];
} RMC_t;


static QueueHandle_t xQueueCmd;
static QueueHandle_t uart0_queue;

static void uart_event_task(void *pvParameters)
{
	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	esp_log_level_set(pcTaskGetTaskName(0), ESP_LOG_WARN);

	uart_event_t event;
	size_t buffered_size;
	uint8_t* rxdata = (uint8_t*) malloc(RX_BUF_SIZE);
	CMD_t cmdBuf;
	cmdBuf.command = CMD_NMEA;
	cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

	for(;;) {
		//Waiting for UART event.
		if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			bzero(rxdata, RX_BUF_SIZE);
			ESP_LOGI(pcTaskGetTaskName(0), "uart[%d] event:", UART_NUM_1);
			switch(event.type) {
				//Event of UART receving data
				/*We'd better handler data event fast, there would be much more data events than
				other types of events. If we take too much time on data event, the queue might
				be full.*/
				case UART_DATA:
					ESP_LOGI(pcTaskGetTaskName(0), "[UART DATA]: %d", event.size);
					break;
				//Event of HW FIFO overflow detected
				case UART_FIFO_OVF:
					ESP_LOGW(pcTaskGetTaskName(0), "hw fifo overflow");
					// If fifo overflow happened, you should consider adding flow control for your application.
					// The ISR has already reset the rx FIFO,
					// As an example, we directly flush the rx buffer here in order to read more data.
					uart_flush_input(UART_NUM_1);
					xQueueReset(uart0_queue);
					break;
				//Event of UART ring buffer full
				case UART_BUFFER_FULL:
					ESP_LOGW(pcTaskGetTaskName(0), "ring buffer full");
					// If buffer full happened, you should consider encreasing your buffer size
					// As an example, we directly flush the rx buffer here in order to read more data.
					uart_flush_input(UART_NUM_1);
					xQueueReset(uart0_queue);
					break;
				//Event of UART RX break detected
				case UART_BREAK:
					ESP_LOGW(pcTaskGetTaskName(0), "uart rx break");
					break;
				//Event of UART parity check error
				case UART_PARITY_ERR:
					ESP_LOGW(pcTaskGetTaskName(0), "uart parity error");
					break;
				//Event of UART frame error
				case UART_FRAME_ERR:
					ESP_LOGW(pcTaskGetTaskName(0), "uart frame error");
					break;
				//UART_PATTERN_DET
				case UART_PATTERN_DET:
					uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
					int pos = uart_pattern_pop_pos(UART_NUM_1);
					ESP_LOGI(pcTaskGetTaskName(0), "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
					if (pos == -1) {
						// There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
						// record the position. We should set a larger queue size.
						// As an example, we directly flush the rx buffer here.
						uart_flush_input(UART_NUM_1);
					} else {
						uart_read_bytes(UART_NUM_1, rxdata, buffered_size, 100 / portTICK_PERIOD_MS);
						// Data ends with 0x0d 0x0a
						ESP_LOGI(pcTaskGetTaskName(0), "rxdata=[%s]", rxdata);
						ESP_LOGI(pcTaskGetTaskName(0), "rxdata=0x%x", rxdata[buffered_size-2]);
						ESP_LOGI(pcTaskGetTaskName(0), "rxdata=0x%x", rxdata[buffered_size-1]);
						//cmdBuf.length = buffered_size;
						//memcpy((char *)cmdBuf.payload, (char *)rxdata, buffered_size); 
						//cmdBuf.payload[buffered_size] = 0;
						// Delete 0x0d 0x0a
						cmdBuf.length = buffered_size - 2;
						memcpy((char *)cmdBuf.payload, (char *)rxdata, buffered_size - 2); 
						cmdBuf.payload[buffered_size - 2] = 0;
						xQueueSend(xQueueCmd, &cmdBuf, 0);
					}
					break;
				//Others
				default:
					ESP_LOGW(pcTaskGetTaskName(0), "uart event type: %d", event.type);
					break;
			}
		}
	}
	// never reach
	free(rxdata);
	rxdata = NULL;
	vTaskDelete(NULL);
}

void buttonA(void *pvParameters)
{
	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	CMD_t cmdBuf;
	cmdBuf.command = CMD_START_RECORD;
	cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

	// set the GPIO as a input
	gpio_pad_select_gpio(GPIO_INPUT_A);
	gpio_set_direction(GPIO_INPUT_A, GPIO_MODE_DEF_INPUT);

	while(1) {
		int level = gpio_get_level(GPIO_INPUT_A);
		if (level == 0) {
			ESP_LOGI(pcTaskGetTaskName(0), "Push Button");
			while(1) {
				level = gpio_get_level(GPIO_INPUT_A);
				if (level == 1) break;
				vTaskDelay(1);
			}

			// Post an item to the front of a queue.
			if (xQueueSendToFront(xQueueCmd, &cmdBuf, portMAX_DELAY) != pdPASS) {
				ESP_LOGE(pcTaskGetTaskName(0), "xQueueSend Fail");
			}
			if (cmdBuf.command == CMD_START_RECORD) {
				cmdBuf.command = CMD_STOP_RECORD;
			} else {
				cmdBuf.command = CMD_START_RECORD;
			}
		}
		vTaskDelay(1);
	}
}

void buttonB(void *pvParameters)
{
	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	CMD_t cmdBuf;
	//cmdBuf.command = CMD_START_PLAYBACK_REPEAT;
	cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

	// set the GPIO as a input
	gpio_pad_select_gpio(GPIO_INPUT_B);
	gpio_set_direction(GPIO_INPUT_B, GPIO_MODE_DEF_INPUT);

	while(1) {
		int level = gpio_get_level(GPIO_INPUT_B);
		if (level == 0) {
			ESP_LOGI(pcTaskGetTaskName(0), "Push Button");
			TickType_t startTick = xTaskGetTickCount();
			while(1) {
				level = gpio_get_level(GPIO_INPUT_B);
				if (level == 1) break;
				vTaskDelay(1);
			}
			TickType_t endTick = xTaskGetTickCount();
			TickType_t diffTick = endTick-startTick;
			cmdBuf.command = CMD_START_PLAYBACK_REPEAT;
			if (diffTick > 200) cmdBuf.command = CMD_START_PLAYBACK_ONCE;

			// Post an item to the front of a queue.
			if (xQueueSendToFront(xQueueCmd, &cmdBuf, portMAX_DELAY) != pdPASS) {
				ESP_LOGE(pcTaskGetTaskName(0), "xQueueSend Fail");
			}
		}
		vTaskDelay(1);
	}
}

void buttonC(void *pvParameters)
{
	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	CMD_t cmdBuf;
	cmdBuf.command = CMD_STOP_PLAYBACK;
	cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

	// set the GPIO as a input
	gpio_pad_select_gpio(GPIO_INPUT_C);
	gpio_set_direction(GPIO_INPUT_C, GPIO_MODE_DEF_INPUT);

	while(1) {
		int level = gpio_get_level(GPIO_INPUT_C);
		if (level == 0) {
			ESP_LOGI(pcTaskGetTaskName(0), "Push Button");
			while(1) {
				level = gpio_get_level(GPIO_INPUT_C);
				if (level == 1) break;
				vTaskDelay(1);
			}

			// Post an item to the front of a queue.
			if (xQueueSendToFront(xQueueCmd, &cmdBuf, portMAX_DELAY) != pdPASS) {
				ESP_LOGE(pcTaskGetTaskName(0), "xQueueSend Fail");
			}
		}
		vTaskDelay(1);
	}
}

static void send_timercb(void *timer)
{
	ESP_LOGD(TAG, "send_timercb");
	CMD_t cmdBuf;
	cmdBuf.command = CMD_TIMER;
	//cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();
	xQueueSend(xQueueCmd, &cmdBuf, 0);
}

static void SPIFFS_Directory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(TAG,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}


bool openLogWrite(LOG_t * log) {
	strcpy(log->fname, "/spiffs/nmea.log");
	log->f = fopen(log->fname, "w");
	if (log->f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
		return false; 
	} else {
		ESP_LOGI(TAG, "Success to open file for writing");
		log->record = 0;
		return true; 
	}
}

bool openLogRead(LOG_t * log, uint16_t mode) {
	strcpy(log->fname, "/spiffs/nmea.log");
	log->f = fopen(log->fname, "r");
	if (log->f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for reading");
		return false; 
	} else {
		ESP_LOGI(TAG, "Success to open file for reading");
		log->mode = mode;
		return true; 
	}
}

bool existLog(LOG_t *log) {
	strcpy(log->fname, "/spiffs/nmea.log");
	struct stat st;
	if (stat(log->fname, &st) == 0) {
		return true;
	} else {
		return false;
	}
}


bool readLog(LOG_t *log, uint8_t * payload, size_t * length, size_t payloadSize) {
	if (fgets((char *)payload, payloadSize, log->f) != NULL) {
		*length = strlen((char *)payload) - 1;
		payload[*length] = 0;
		ESP_LOGI(TAG,"readLog payload=[%s] length=%d", (char*)payload, *length);
		return true;
	} else {
		ESP_LOGW(TAG, "End of file reached");
		return false;
	}
}

bool writeLog(LOG_t *log, uint8_t * payload, size_t length) {
	if (fprintf(log->f, "%s\n",(char *) payload) == (length+1)) {
		ESP_LOGD(TAG, "Success to write");
		log->record++;
		return true;
	} else {
		ESP_LOGE(TAG, "Failed to write");
		return false;
	}
}

bool closeLog(LOG_t *log) {
	fclose(log->f);
	ESP_LOGI(TAG, "log->record=%d", log->record);
	return true;
}


bool parse_nmea_rmc(RMC_t *rmc, uint8_t * payload, size_t length) {
	int typeLength = 0;
	for(int i=0;i<length;i++) {
		if (payload[i] == ',') break;
		typeLength++; 
	}
	ESP_LOGD(TAG, "[%s] typeString=%.*s", __FUNCTION__, typeLength, payload);
	if (strncmp((char *)payload, "$GPRMC", typeLength) != 0) return false;

	ESP_LOGD(TAG, "[%s] payload=%.*s", __FUNCTION__, length, payload);
	int index = 0;
	int offset = 0;
	rmc->_time[0] = 0;
	rmc->_valid[0] = 0;
	rmc->_lat1[0] = 0;
	rmc->_lat2[0] = 0;
	rmc->_lon1[0] = 0;
	rmc->_lon2[0] = 0;
	rmc->_speed[0] = 0;
	rmc->_orient[0] = 0;
	rmc->_date[0] = 0;
	for(int i=0;i<length;i++) {
		if (payload[i] == ',') {
			index++;
			offset = 0;
		} else {
			if (index == 0) {

			} else if (index == 1) {
				rmc->_time[offset++] = payload[i];
				rmc->_time[offset] = 0;

			} else if (index == 2) {
				rmc->_valid[offset++] = payload[i];
				rmc->_valid[offset] = 0;

			} else if (index == 3) {
				rmc->_lat1[offset++] = payload[i];
				rmc->_lat1[offset] = 0;

			} else if (index == 4) {
				rmc->_lat2[offset++] = payload[i];
				rmc->_lat2[offset] = 0;

			} else if (index == 5) {
				rmc->_lon1[offset++] = payload[i];
				rmc->_lon1[offset] = 0;

			} else if (index == 6) {
				rmc->_lon2[offset++] = payload[i];
				rmc->_lon2[offset] = 0;

			} else if (index == 7) {
				rmc->_speed[offset++] = payload[i];
				rmc->_speed[offset] = 0;

			} else if (index == 8) {
				rmc->_orient[offset++] = payload[i];
				rmc->_orient[offset] = 0;

			} else if (index == 9) {
				rmc->_date[offset++] = payload[i];
				rmc->_date[offset] = 0;
			}
		}
	}
	return true;
}


void tft(void *pvParameters)
{
	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	// set font file
	FontxFile fxG[2];
	InitFontx(fxG,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	FontxFile fxM[2];
	InitFontx(fxM,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fxG, 0, buffer, &fontWidth, &fontHeight);
	ESP_LOGI(pcTaskGetTaskName(0), "fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

	// Setup Screen
	TFT_t dev;
	spi_master_init(&dev, CS_GPIO, DC_GPIO, RESET_GPIO, BL_GPIO);
	lcdInit(&dev, 0x9341, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0);
	ESP_LOGI(pcTaskGetTaskName(0), "Setup Screen done");

	int lines = (SCREEN_HEIGHT - fontHeight) / fontHeight;
	ESP_LOGD(pcTaskGetTaskName(0), "SCREEN_HEIGHT=%d fontHeight=%d lines=%d", SCREEN_HEIGHT, fontHeight, lines);
	int ymax = (lines+1) * fontHeight;
	ESP_LOGD(pcTaskGetTaskName(0), "ymax=%d",ymax);

	// Initial Screen
	//uint8_t ascii[DISPLAY_LENGTH+1];
	uint8_t ascii[44];
	lcdFillScreen(&dev, BLACK);
	lcdSetFontDirection(&dev, 0);

	// Reset scroll area
	lcdSetScrollArea(&dev, 0, 0x0140, 0);

	strcpy((char *)ascii, "NMEA");
	lcdDrawString(&dev, fxG, 0, fontHeight-1, ascii, YELLOW);
	//uint16_t xstatus = 10*fontWidth;
	uint16_t xstatus = 5*fontWidth;
	uint16_t xlatlon = 17*fontWidth;
	strcpy((char *)ascii, "View");
	lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);

	uint16_t vsp = fontHeight*2;
	uint16_t ypos = (fontHeight*2) - 1;
	uint16_t current = 0;
	CMD_t cmdBuf;
	LOG_t log;
	bool isRecord = false;
	bool isPlayback = false;
	uint8_t* txdata = (uint8_t*) malloc(TX_BUF_SIZE);
	TickType_t recordStartTick;
	TickType_t recordEndTick;

	while(1) {
		xQueueReceive(xQueueCmd, &cmdBuf, portMAX_DELAY);
		ESP_LOGD(pcTaskGetTaskName(0),"cmdBuf.command=%d isRecord=%d isPlayback=%d", cmdBuf.command, isRecord, isPlayback);
		if (cmdBuf.command == CMD_START_RECORD) {
			if (isPlayback) continue;
			if (openLogWrite(&log)) {
				lcdDrawFillRect(&dev, xstatus, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
				strcpy((char *)ascii, "Recording");
				lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);
				isRecord = true;
				recordStartTick = xTaskGetTickCount();
			}

		} else if (cmdBuf.command == CMD_STOP_RECORD) {
			if (!isRecord) continue;
			closeLog(&log);
			size_t total = 0, used = 0;
			esp_spiffs_info(NULL, &total, &used);
			ESP_LOGI(pcTaskGetTaskName(0),"Partition size: total: %d, used: %d", total, used);
			SPIFFS_Directory("/spiffs");
			lcdDrawFillRect(&dev, xstatus, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
			strcpy((char *)ascii, "View");
			lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);
			isRecord = false;
			recordEndTick = xTaskGetTickCount();
			TickType_t diffTick = recordEndTick - recordStartTick;
			int recordingTime = (diffTick * portTICK_RATE_MS) / 1000;
			ESP_LOGI(pcTaskGetTaskName(0),"Recording time is %d Sec", recordingTime);

		} else if (cmdBuf.command == CMD_START_PLAYBACK_REPEAT) {
			if (isPlayback) continue;
			if (isRecord) continue;
			if (!existLog(&log)) continue;
			if (openLogRead(&log, CMD_START_PLAYBACK_REPEAT)) {
				lcdDrawFillRect(&dev, xstatus, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
				//lcdDrawFillRect(&dev, 0, fontHeight-1, SCREEN_WIDTH-1, SCREEN_HEIGHT-1, BLACK);
				strcpy((char *)ascii, "Playback-Repeat");
				lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);
				isPlayback = true;
			}

		} else if (cmdBuf.command == CMD_START_PLAYBACK_ONCE) {
			if (isPlayback) continue;
			if (isRecord) continue;
			if (!existLog(&log)) continue;
			if (openLogRead(&log, CMD_START_PLAYBACK_ONCE)) {
				lcdDrawFillRect(&dev, xstatus, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
				//lcdDrawFillRect(&dev, 0, fontHeight-1, SCREEN_WIDTH-1, SCREEN_HEIGHT-1, BLACK);
				strcpy((char *)ascii, "Playback-Once");
				lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);
				isPlayback = true;
			}

		} else if (cmdBuf.command == CMD_STOP_PLAYBACK) {
			if (!isPlayback) continue;
			lcdDrawFillRect(&dev, xstatus, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
			strcpy((char *)ascii, "View");
			lcdDrawString(&dev, fxG, xstatus, fontHeight-1, ascii, RED);
			closeLog(&log);
			isPlayback = false;

		} else if (cmdBuf.command == CMD_NMEA || cmdBuf.command == CMD_TIMER) {
			uint16_t color = CYAN;
			if (cmdBuf.command == CMD_TIMER) vTaskDelay(1); // Avoid WatchDog trigger
			if (cmdBuf.command == CMD_NMEA && isPlayback) continue;
			if (cmdBuf.command == CMD_TIMER && !isPlayback) continue;
			if (cmdBuf.command == CMD_NMEA) {
				RMC_t rmcBuf;
				parse_nmea_rmc(&rmcBuf, cmdBuf.payload, cmdBuf.length);
				ESP_LOGD(pcTaskGetTaskName(0),"_lat1=%s _lat2=%s _lon1=%s _lon2=%s", rmcBuf._lat1, rmcBuf._lat2, rmcBuf._lon1, rmcBuf._lon2);
				if (strlen((char *)rmcBuf._lat1) == 0) {
					lcdDrawFillRect(&dev, xlatlon, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
					strcpy((char *)ascii, "Signal OFF");
					lcdDrawString(&dev, fxG, xlatlon, fontHeight-1, ascii, BLUE);
				} else {
					lcdDrawFillRect(&dev, xlatlon, 0, SCREEN_WIDTH-1, fontHeight-1, BLACK);
					strcpy((char *)ascii, "Signal ON");
					lcdDrawString(&dev, fxG, xlatlon, fontHeight-1, ascii, RED);
				}
				ESP_LOGD(pcTaskGetTaskName(0), "uart payload=[%s]", cmdBuf.payload);
				if (isRecord) {
					writeLog(&log, cmdBuf.payload, cmdBuf.length);
					color = BLUE;
				}
			}
			if (cmdBuf.command == CMD_TIMER) {
				if (!readLog(&log, cmdBuf.payload, &cmdBuf.length, MAX_PAYLOAD)) {
					if (log.mode == CMD_START_PLAYBACK_ONCE) continue;
					ESP_LOGI(pcTaskGetTaskName(0), "rewind file");
					rewind(log.f);
					readLog(&log, cmdBuf.payload, &cmdBuf.length, MAX_PAYLOAD);
				}
				memcpy(txdata, cmdBuf.payload, cmdBuf.length);
				txdata[cmdBuf.length] = 0x0d;
				txdata[cmdBuf.length+1] = 0x0a;
				uart_write_bytes(UART_NUM_1, (const char *) txdata, cmdBuf.length+2);
				color = GREEN;
			}

			int loop = (cmdBuf.length+DISPLAY_LENGTH) / DISPLAY_LENGTH;
			int index = 0;
			for(int i=0;i<loop;i++) {
				memcpy((char *)ascii, (char *)&(cmdBuf.payload[index]), DISPLAY_LENGTH);
				ascii[DISPLAY_LENGTH] = 0;
				index = index + DISPLAY_LENGTH;

				if (current < lines) {
					//lcdDrawString(&dev, fxM, 0, ypos, cmdBuf.payload, CYAN);
					lcdDrawString(&dev, fxM, 0, ypos, ascii, color);
				} else {
					lcdDrawFillRect(&dev, 0, ypos-fontHeight, SCREEN_WIDTH-1, ypos, BLACK);
					lcdSetScrollArea(&dev, fontHeight, (SCREEN_HEIGHT-fontHeight), 0);
					lcdScroll(&dev, vsp);
					vsp = vsp + fontHeight;
					if (vsp > ymax) vsp = fontHeight*2;
					//lcdDrawString(&dev, fxM, 0, ypos, cmdBuf.payload, CYAN);
					lcdDrawString(&dev, fxM, 0, ypos, ascii, color);
				}
				current++;
				ypos = ypos + fontHeight;
				if (ypos > ymax) ypos = (fontHeight*2) - 1;
			}

		}
	}

	// nerver reach
	while (1) {
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

void app_main()
{
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	// Initialize SPIFFS
	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 8,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	ret = esp_vfs_spiffs_register(&conf);
	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	SPIFFS_Directory("/spiffs");
	ESP_LOGI(TAG, "Initializing SPIFFS done");

	ESP_LOGI(TAG, "Initializing UART");
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_1, &uart_config);

	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(UART_NUM_1, CONFIG_UART_TXD_GPIO, CONFIG_UART_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Install UART driver, and get the queue.
	uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 20, &uart0_queue, 0);

	//Set uart pattern detect function.
	//uart_enable_pattern_det_intr(UART_NUM_1, 0x0a, 1, 10000, 10, 10); // pattern is LF
	uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 9, 0, 0); // pattern is LF
	//uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 10000, 10, 10); // pattern is LF
	//Reset the pattern queue length to record at most 20 pattern positions.
	uart_pattern_queue_reset(UART_NUM_1, 20);
	ESP_LOGI(TAG, "Initializing UART done");


	// Create Queue
	xQueueCmd = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueCmd );

	// Create task
	xTaskCreate(buttonA, "BUTTON-A", 1024*2, NULL, 2, NULL);
	xTaskCreate(buttonB, "BUTTON-B", 1024*2, NULL, 2, NULL);
	xTaskCreate(buttonC, "BUTTON-C", 1024*2, NULL, 2, NULL);
	xTaskCreate(tft, "TFT", 1024*4, NULL, 5, NULL);
	//Create a task to handler UART event from ISR
	xTaskCreate(uart_event_task, "uart_event", 1024*4, NULL, 5, NULL);

	// Create timer
	TimerHandle_t xTimer = xTimerCreate("TIMER", CONFIG_PLAYBACK_PERIOD / portTICK_RATE_MS, true, NULL, send_timercb);
	//TimerHandle_t xTimer = xTimerCreate("TIMER", 1000 / portTICK_RATE_MS, true, NULL, send_timercb);
	xTimerStart(xTimer, 0);

}

