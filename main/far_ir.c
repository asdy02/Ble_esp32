/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "esp_sleep.h"



#define GATTS_TAG "ZT_GATTS"

#define VERSION_MARJOR   1

#define VERSION_MINOR    4

#define VERSION_SUB      0

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void ZT_Bt_init(void);
static void ZT_Bt_deinit(void);


#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define TEST_DEVICE_NAME            "HBeauty"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

//work timer start
#define default_work_time (30ll*60*1000*1000)

static bool work_timer_run = false;

static bool work_overtemp_stop = false;

static bool work_overtemp_reopen = false;

static uint64_t work_time =  default_work_time;

static uint64_t work_overtemp_reopen_time =  (5ll*60*1000*1000);

static uint64_t work_time_start;

static uint64_t work_time_end;

static uint16_t offline_work_count =  0;

static uint64_t offline_work_time[] = {0};



	
esp_timer_handle_t periodic_timer;

static void periodic_timer_callback(void* arg);

const esp_timer_create_args_t periodic_timer_args = {
		.callback = &periodic_timer_callback,
		/* name is optional, but may help identify the timer when debugging */
		.name = "periodic"
};

esp_timer_handle_t oneshot_timer;

static void work_timer_callback(void* arg);

const esp_timer_create_args_t work_timer_args = {
		.callback = &work_timer_callback,
		/* argument specified here will be passed to timer callback function */
		//.arg = (void*) oneshot_timer,
		.name = "work-timer"
};
		
//BT stat
static bool Bt_connect = false;

static bool Bt_open = false;

static bool Bt_notify_on = false;

static bool Bt_responce_on = false;

esp_gatt_if_t zt_gatt_if;

esp_ble_gatts_cb_param_t *zt_param;



//GPIO DEFINE START
#define GPIO_OUTPUT_LED_R      5
#define GPIO_OUTPUT_LED_G      18
#define GPIO_OUTPUT_LED_B      19
#define GPIO_OUTPUT_HEAT_NO    21
#define GPIO_OUTPUT_PWR_CTL    23
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_LED_R) | (1ULL<<GPIO_OUTPUT_LED_G) | (1ULL<<GPIO_OUTPUT_LED_B) | (1ULL<<GPIO_OUTPUT_PWR_CTL)\
                              | (1ULL<<GPIO_OUTPUT_HEAT_NO))

#define GPIO_INPUT_VDET_KEY     22
#define GPIO_INPUT_TEMP         26
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_VDET_KEY) | (1ULL<<GPIO_INPUT_TEMP))
#define ESP_INTR_FLAG_DEFAULT 0

static uint8_t HEAT_RELAY_STAT = 0;

typedef struct {
	bool press;
    uint8_t pressed_num;	
}key_stat_t;

static uint64_t key_press_time = 0;

static uint64_t key_release_time = 0;

static uint64_t key_press_count = 0;

typedef struct {
	uint8_t work_stat;
		
}machine_data_t;


enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

typedef enum {
    SMP_CMD_GET_DEVICE_MODEL = 0x1001,
    SMP_CMD_GET_WORKING_TIME = 0x1002,
    SMP_CMD_REPORT_RECORD_WORK_STATE = 0x1003,
    SMP_CMD_SET_RECORD_WORKING_TIME = 0x1004,
	SMP_CMD_SET_WORKING_TIME = 0x2001,
	SMP_CMD_HEART = 0x0000,
	SMP_CMD_SET_DEBUG_OUTPUT = 0x3001,
	SMP_CMD_SET_WORK_STATE = 0x3002,
	SMP_MSG_DEBUG = 0x4001,
	SMP_CMD_REPORT_WORK_STATE = 0x4002
}CMD_id;

machine_data_t m_stat;

key_stat_t key_stat;

static void set_work(uint8_t state)
{
  if(state == 1)
  	{
  	  gpio_set_level(GPIO_OUTPUT_HEAT_NO, 1);
	  HEAT_RELAY_STAT = 1;

	  if (work_timer_run == false)
		 {
		  ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  work_timer_run = true;
		 }
		 else
		 {
		  ESP_ERROR_CHECK(esp_timer_stop(oneshot_timer));
          ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  work_timer_run = true;
		 }
  	}
  else if(state == 0)
  	{
  	  gpio_set_level(GPIO_OUTPUT_HEAT_NO, 0);
	  HEAT_RELAY_STAT = 0;

	  	 if (work_timer_run == false)
		 {
		  //ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  //work_timer_run = true;
		 }
		 else
		 {
		  ESP_ERROR_CHECK(esp_timer_stop(oneshot_timer));
          //ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  work_timer_run = false;
		 }
  	}
}

static uint8_t get_work_stat(void)
{
 
	 return HEAT_RELAY_STAT ;
 	
}

static void set_power(uint8_t state)
{
  if(state == 1)
  	{
  	  gpio_set_level(GPIO_OUTPUT_PWR_CTL, 1);
  	}
  else if(state == 0)
  	{
  	  gpio_set_level(GPIO_OUTPUT_PWR_CTL, 0);
  	}
}


static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
	uint64_t temp;
	
	
	for (; ; )
		{
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
			{
			printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
	
			if (io_num == 22)
				{
				//vTaskDelay(500 / portTICK_RATE_MS);
				if (!gpio_get_level(io_num) && key_stat.press == false)
					{
					key_stat.press		= true;
					key_press_time		= esp_timer_get_time();
	
					}
	
				if (gpio_get_level(io_num) && key_stat.press == true)
					{
	
					key_release_time	= esp_timer_get_time();
	
					temp				= key_release_time - key_press_time;
	
					ESP_LOGI(GATTS_TAG, "key press time : %lld us", temp);
	
					if (temp >= 10000000 && temp < 15000000)
						{
	
						gpio_set_level(GPIO_OUTPUT_PWR_CTL, 0);
						}
					else if (temp >= 100000 && temp < 1000000)
						{

						#if 0
						if (HEAT_RELAY_STAT == 1)
							{
							gpio_set_level(GPIO_OUTPUT_HEAT_NO, 0);
							HEAT_RELAY_STAT 	= 0;
							printf("set HEAT relay off:%d\n", HEAT_RELAY_STAT);
							}
						else 
							{
							gpio_set_level(GPIO_OUTPUT_HEAT_NO, 1);
							HEAT_RELAY_STAT 	= 1;
	
							//ESP_ERROR_CHECK(esp_timer_stop(oneshot_timer));
							if (work_timer_run == false)
								{
								ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
								work_timer_run		= true;
								}
	
							printf("set HEAT relay on:%d\n", HEAT_RELAY_STAT);
							}
					    #endif
						   key_press_count++;

				           if(key_press_count == 2 && Bt_open == false)
				           	{

                              //ZT_Bt_init();

						    }
						   
						}
					else if (temp >= 15000000 && temp <= 30000000)
						{
                        /*
						gpio_set_level(GPIO_OUTPUT_LED_B, 0);
						gpio_set_level(GPIO_OUTPUT_LED_R, 0);
						
						for (int i = 0; i < 5; i++)
							{
							gpio_set_level(GPIO_OUTPUT_LED_G, 1);
							vTaskDelay(800 / portTICK_RATE_MS);
							gpio_set_level(GPIO_OUTPUT_LED_G, 0);
							vTaskDelay(800 / portTICK_RATE_MS);
							ESP_LOGI(GATTS_TAG, "reset %d\n", i);
							}
	
	
						   gpio_set_level(GPIO_OUTPUT_LED_B, 0);
						   gpio_set_level(GPIO_OUTPUT_LED_R, 0);
						   gpio_set_level(GPIO_OUTPUT_LED_G, 1);
	
	
	
						if (work_timer_run == false)
							{
							ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, default_work_time));
							work_timer_run		= true;
							}
	
						else 
							{
							ESP_ERROR_CHECK(esp_timer_stop(oneshot_timer));
							ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, default_work_time));
							work_timer_run		= true;
							}
						ZT_Bt_deinit();
						*/

						//system_restart();

						esp_restart();
						
                        ESP_LOGI(GATTS_TAG, "reset finish\n");
	
						}
					else if (temp >= 5000000 && temp < 10000000)
						{
						ZT_Bt_init();
						}
	
					key_stat.press		= false;
					}
	
	              
				}
			}
		}
	
	
	}
//GPIO DEFINE END

//ADC DEFINE START

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_9;     //GPIO34 if ADC1, GPIO26 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_2;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
//ADC DEFINE END




uint8_t char1_str[] = {0x11,0x22,0x33};
esp_gatt_char_prop_t a_property = 0;
esp_gatt_char_prop_t b_property = 0;

esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_b_event_handler,                   /* This demo does not implement, similar as profile A */
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static uint16_t rev_write_cmd = 0xffff;
static bool rev_write = false;

static void bt_task_notify(void* arg)
{

esp_gatt_rsp_t *rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
int64_t time_msec;
esp_gatt_status_t status = ESP_GATT_OK;
//uint16_t cmd = 0;
int32_t temp;
esp_err_t response_err;

for (; ; )
{

    printf("bt_notify cmd: %d\n",rev_write_cmd);

 //vTaskDelay(pdMS_TO_TICKS(500));
 //if(rev_write == true)
 //if(1)
 if(Bt_notify_on == true && Bt_connect == true)
 {
	
	switch(rev_write_cmd){
		
		 case SMP_CMD_GET_DEVICE_MODEL:
	
	
			memset(rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp->attr_value.handle = zt_param->read.handle;
			rsp->attr_value.len = 21;
			/*
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			*/
			//session id
			rsp->attr_value.value[0] = zt_param->write.value[0];
			rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

	
			//cmd
			rsp->attr_value.value[2] = SMP_CMD_GET_DEVICE_MODEL & 0xFF;
			rsp->attr_value.value[3] = (SMP_CMD_GET_DEVICE_MODEL >> 8) & 0xFF;
	
			//reserved
			rsp->attr_value.value[4] = 0x00;
			rsp->attr_value.value[5] = 0x00;	
			rsp->attr_value.value[6] = 0x00;		
			rsp->attr_value.value[7] = 0x00;
			rsp->attr_value.value[8] = 0x00;
			rsp->attr_value.value[9] = 0x00;
			rsp->attr_value.value[10] = 0x00;
			rsp->attr_value.value[11] = 0x00;
	
			//datalen
			rsp->attr_value.value[12] = 0x05;
			rsp->attr_value.value[13] = 0x00;
			rsp->attr_value.value[14] = 0x00;
			rsp->attr_value.value[15] = 0x00;
	
			//data
			rsp->attr_value.value[16] = 0x00;
			rsp->attr_value.value[17] = 0x00;
			rsp->attr_value.value[18] = 0x00;
			rsp->attr_value.value[19] = 0x00;
			rsp->attr_value.value[20] = 0x11;
	
			printf("response for cmd:SMP_CMD_GET_DEVICE_MODEL\n");

			/*
			if(rsp->attr_value.len <= 20)
			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);
			else{

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						20, rsp->attr_value.value, false);

			vTaskDelay(pdMS_TO_TICKS(200));

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);

			}*/
			
			 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);

		
			 break;
	
	
		 case SMP_CMD_GET_WORKING_TIME:
	
			 time_msec = esp_timer_get_time();
	
			 
			 memset(rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp->attr_value.handle = zt_param->read.handle;
			rsp->attr_value.len = 24;
			/*
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			*/
			//session id
			rsp->attr_value.value[0] = zt_param->write.value[0];
			rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

	
			//cmd
			rsp->attr_value.value[2] = SMP_CMD_GET_WORKING_TIME & 0xFF;
			rsp->attr_value.value[3] = (SMP_CMD_GET_WORKING_TIME >> 8) & 0xFF;
	
			//reserved
			rsp->attr_value.value[4] = 0x00;
			rsp->attr_value.value[5] = 0x00;	
			rsp->attr_value.value[6] = 0x00;		
			rsp->attr_value.value[7] = 0x00;
			rsp->attr_value.value[8] = 0x00;
			rsp->attr_value.value[9] = 0x00;
			rsp->attr_value.value[10] = 0x00;
			rsp->attr_value.value[11] = 0x00;
	
			//datalen
			rsp->attr_value.value[12] = 0x08;
			rsp->attr_value.value[13] = 0x00;
			rsp->attr_value.value[14] = 0x00;
			rsp->attr_value.value[15] = 0x00;
	
			//data
			temp = work_time/(1000*1000);
			rsp->attr_value.value[16] = 0x00;
			rsp->attr_value.value[17] = 0x00;
			rsp->attr_value.value[18] = 0x00;
			rsp->attr_value.value[19] = 0x00;
			rsp->attr_value.value[20] = temp & 0xFF;
			rsp->attr_value.value[21] = (temp >> 8) & 0xFF;
			rsp->attr_value.value[22] = (temp >> 16) & 0xFF;
			rsp->attr_value.value[23] = (temp >> 24) & 0xFF;

	
			printf("response for cmd:SMP_CMD_GET_WORKING_TIME time_msec = %lld\n",time_msec);

			/*
			if(rsp->attr_value.len <= 20)
			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);
			else{

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						20, rsp->attr_value.value, false);

			vTaskDelay(pdMS_TO_TICKS(200));

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);

			}*/
			 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);
			 break;
	
		 case SMP_CMD_REPORT_RECORD_WORK_STATE:
	
	
			 break;
	
		 case SMP_CMD_SET_RECORD_WORKING_TIME:

		   memset(rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp->attr_value.handle = zt_param->read.handle;
			rsp->attr_value.len = 20;
			/*
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			*/
			//session id
			rsp->attr_value.value[0] = zt_param->write.value[0];
			rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

	
			//cmd
			rsp->attr_value.value[2] = SMP_CMD_SET_RECORD_WORKING_TIME & 0xFF;
			rsp->attr_value.value[3] = (SMP_CMD_SET_RECORD_WORKING_TIME >> 8) & 0xFF;
	
			//reserved
			rsp->attr_value.value[4] = 0x00;
			rsp->attr_value.value[5] = 0x00;	
			rsp->attr_value.value[6] = 0x00;		
			rsp->attr_value.value[7] = 0x00;
			rsp->attr_value.value[8] = 0x00;
			rsp->attr_value.value[9] = 0x00;
			rsp->attr_value.value[10] = 0x00;
			rsp->attr_value.value[11] = 0x00;
	
			//datalen
			rsp->attr_value.value[12] = 0x04;
			rsp->attr_value.value[13] = 0x00;
			rsp->attr_value.value[14] = 0x00;
			rsp->attr_value.value[15] = 0x00;
	
			//data
			rsp->attr_value.value[16] = 0x00;
			rsp->attr_value.value[17] = 0x00;
			if(offline_work_time[(zt_param->write.value[17] << 8) + zt_param->write.value[17]] > 0)
	        {
			rsp->attr_value.value[18] = 0x00;
			rsp->attr_value.value[19] = 0x00;
			}
			else{

            rsp->attr_value.value[18] = 0x01;
			rsp->attr_value.value[19] = 0x00;
			}
			
	
			printf("response for cmd:SMP_CMD_GET_DEVICE_MODEL\n");

			/*
			if(rsp->attr_value.len <= 20)
			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);
			else{

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						20, rsp->attr_value.value, false);

			vTaskDelay(pdMS_TO_TICKS(200));

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);

			}*/
			 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);

		
			 break;
	
		 case SMP_CMD_SET_WORKING_TIME:
			 
			memset(rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp->attr_value.handle = zt_param->read.handle;
			rsp->attr_value.len = 20;
			/*
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			*/
			//session id
			rsp->attr_value.value[0] = zt_param->write.value[0];
			rsp->attr_value.value[1] = zt_param->write.value[1]<<8;


	
			//cmd
			rsp->attr_value.value[2] = SMP_CMD_SET_WORKING_TIME & 0xFF;
			rsp->attr_value.value[3] = (SMP_CMD_SET_WORKING_TIME >> 8) & 0xFF;
	
			//reserved
			rsp->attr_value.value[4] = 0x00;
			rsp->attr_value.value[5] = 0x00;	
			rsp->attr_value.value[6] = 0x00;		
			rsp->attr_value.value[7] = 0x00;
			rsp->attr_value.value[8] = 0x00;
			rsp->attr_value.value[9] = 0x00;
			rsp->attr_value.value[10] = 0x00;
			rsp->attr_value.value[11] = 0x00;
	
			//datalen
			rsp->attr_value.value[12] = 0x04;
			rsp->attr_value.value[13] = 0x00;
			rsp->attr_value.value[14] = 0x00;
			rsp->attr_value.value[15] = 0x00;
	
			//data
			rsp->attr_value.value[16] = 0x00;
			rsp->attr_value.value[17] = 0x00;
			rsp->attr_value.value[18] = 0x00;
			rsp->attr_value.value[19] = 0x00;

			//rsp->attr_value.value[20] = 0xFF;
			//rsp->attr_value.value[21] = 0xFF;
		
	
			printf("response for cmd:SMP_CMD_SET_WORKING_TIME time_msec = %lld\n",work_time);
			//esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
										//ESP_GATT_OK, rsp);
			
			/*
			if(rsp->attr_value.len <= 20)
			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);
			else{

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						20, rsp->attr_value.value, false);

			vTaskDelay(pdMS_TO_TICKS(200));

			esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);

			}*/
			
			 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						rsp->attr_value.len, rsp->attr_value.value, false);

			 break;
	
		 case SMP_CMD_HEART:
		 	
		 memset(rsp, 0, sizeof(esp_gatt_rsp_t));
		 rsp->attr_value.handle = zt_param->read.handle;
		 rsp->attr_value.len = 20;
		 /*
		 rsp.attr_value.value[0] = 0xde;
		 rsp.attr_value.value[1] = 0xed;
		 rsp.attr_value.value[2] = 0xbe;
		 rsp.attr_value.value[3] = 0xef;
		 */
		 //session id
		 rsp->attr_value.value[0] = zt_param->write.value[0];
		 rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

		 
		 
		 //cmd
		 rsp->attr_value.value[2] = SMP_CMD_HEART & 0xFF;
		 rsp->attr_value.value[3] = (SMP_CMD_HEART >> 8) & 0xFF;
		 
		 //reserved
		 rsp->attr_value.value[4] = 0x00;
		 rsp->attr_value.value[5] = 0x00;	 
		 rsp->attr_value.value[6] = 0x00;		 
		 rsp->attr_value.value[7] = 0x00;
		 rsp->attr_value.value[8] = 0x00;
		 rsp->attr_value.value[9] = 0x00;
		 rsp->attr_value.value[10] = 0x00;
		 rsp->attr_value.value[11] = 0x00;
		 
		 //datalen
		 rsp->attr_value.value[12] = 0x04;
		 rsp->attr_value.value[13] = 0x00;
		 rsp->attr_value.value[14] = 0x00;
		 rsp->attr_value.value[15] = 0x00;
		 
		 //data
		 rsp->attr_value.value[16] = 0x00;
		 rsp->attr_value.value[17] = 0x00;
		 rsp->attr_value.value[18] = 0x00;
		 rsp->attr_value.value[19] = 0x00;
		 
		 
		 printf("response for cmd:SMP_CMD_HEART \n");
		 
		 /*
		 if(rsp->attr_value.len <= 20)
		 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
					 rsp->attr_value.len, rsp->attr_value.value, false);
		 else{
		 
		 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
					 20, rsp->attr_value.value, false);
		 
		 vTaskDelay(pdMS_TO_TICKS(200));
		 
		 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
					 rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);
		 
		 }*/
		 
		  esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
					 rsp->attr_value.len, rsp->attr_value.value, false);


			 break;
	
		 case SMP_CMD_SET_DEBUG_OUTPUT:
	
	                memset(rsp, 0, sizeof(esp_gatt_rsp_t));
					rsp->attr_value.handle = zt_param->read.handle;
					rsp->attr_value.len = 20;
					
					/*
					rsp.attr_value.value[0] = 0xde;
					rsp.attr_value.value[1] = 0xed;
					rsp.attr_value.value[2] = 0xbe;
					rsp.attr_value.value[3] = 0xef;
					*/
					//session id
					rsp->attr_value.value[0] = zt_param->write.value[0];
					rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

					
					
					
					//cmd
					rsp->attr_value.value[2] = SMP_CMD_SET_DEBUG_OUTPUT & 0xFF;
					rsp->attr_value.value[3] = (SMP_CMD_SET_DEBUG_OUTPUT >> 8) & 0xFF;
					
					//reserved
					rsp->attr_value.value[4] = 0x00;
					rsp->attr_value.value[5] = 0x00;
					rsp->attr_value.value[6] = 0x00;
					rsp->attr_value.value[7] = 0x00;
					rsp->attr_value.value[8] = 0x00;
					rsp->attr_value.value[9] = 0x00;
					rsp->attr_value.value[10] = 0x00;
					rsp->attr_value.value[11] = 0x00;
					
					//datalen
					rsp->attr_value.value[12] = 0x04;
					rsp->attr_value.value[13] = 0x00;
					rsp->attr_value.value[14] = 0x00;
					rsp->attr_value.value[15] = 0x00;
					
					//data
					rsp->attr_value.value[16] = 0x00;
					rsp->attr_value.value[17] = 0x00;
					rsp->attr_value.value[18] = 0x00;
					rsp->attr_value.value[19] = 0x00;
					
					//rsp->attr_value.value[20] = 0x11;
					printf("response for cmd:SMP_CMD_SET_WORK_STATE\n");
					
					/*
					if(rsp->attr_value.len <= 20)
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);
					else{
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								20, rsp->attr_value.value, false);
					
					vTaskDelay(pdMS_TO_TICKS(200));
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);
					
					}*/
					
					 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);

					
			 break;
	
		 case SMP_CMD_SET_WORK_STATE:

		 
					memset(rsp, 0, sizeof(esp_gatt_rsp_t));
					rsp->attr_value.handle = zt_param->read.handle;
					rsp->attr_value.len = 20;
					
					/*
					rsp.attr_value.value[0] = 0xde;
					rsp.attr_value.value[1] = 0xed;
					rsp.attr_value.value[2] = 0xbe;
					rsp.attr_value.value[3] = 0xef;
					*/
					//session id
					rsp->attr_value.value[0] = zt_param->write.value[0];
					rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

					
					
					
					//cmd
					rsp->attr_value.value[2] = SMP_CMD_SET_WORK_STATE & 0xFF;
					rsp->attr_value.value[3] = (SMP_CMD_SET_WORK_STATE >> 8) & 0xFF;
					
					//reserved
					rsp->attr_value.value[4] = 0x00;
					rsp->attr_value.value[5] = 0x00;
					rsp->attr_value.value[6] = 0x00;
					rsp->attr_value.value[7] = 0x00;
					rsp->attr_value.value[8] = 0x00;
					rsp->attr_value.value[9] = 0x00;
					rsp->attr_value.value[10] = 0x00;
					rsp->attr_value.value[11] = 0x00;
					
					//datalen
					rsp->attr_value.value[12] = 0x04;
					rsp->attr_value.value[13] = 0x00;
					rsp->attr_value.value[14] = 0x00;
					rsp->attr_value.value[15] = 0x00;
					
					//data
					rsp->attr_value.value[16] = 0x00;
					rsp->attr_value.value[17] = 0x00;
					rsp->attr_value.value[18] = 0x00;
					rsp->attr_value.value[19] = 0x00;
					
					//rsp->attr_value.value[20] = 0x11;
					printf("response for cmd:SMP_CMD_SET_WORK_STATE\n");
					
					/*
					if(rsp->attr_value.len <= 20)
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);
					else{
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								20, rsp->attr_value.value, false);
					
					vTaskDelay(pdMS_TO_TICKS(200));
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);
					
					}*/
					
					 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);

					
						
			 break;
	
		 case SMP_MSG_DEBUG:
	
	
			 break;
	
		case SMP_CMD_REPORT_WORK_STATE:
	
	
					memset(rsp, 0, sizeof(esp_gatt_rsp_t));
					rsp->attr_value.handle = zt_param->read.handle;
					rsp->attr_value.len = 17;
			 
					/*
					rsp.attr_value.value[0] = 0xde;
					rsp.attr_value.value[1] = 0xed;
					rsp.attr_value.value[2] = 0xbe;
					rsp.attr_value.value[3] = 0xef;
					*/
					//session id
					rsp->attr_value.value[0] = zt_param->write.value[0];
					rsp->attr_value.value[1] = zt_param->write.value[1]<<8;

	
			 
					//cmd
					rsp->attr_value.value[2] = SMP_CMD_REPORT_WORK_STATE & 0xFF;
					rsp->attr_value.value[3] = (SMP_CMD_REPORT_WORK_STATE >> 8) & 0xFF;
			 
					//reserved
					rsp->attr_value.value[4] = 0x00;
					rsp->attr_value.value[5] = 0x00;
					rsp->attr_value.value[6] = 0x00;
					rsp->attr_value.value[7] = 0x00;
					rsp->attr_value.value[8] = 0x00;
					rsp->attr_value.value[9] = 0x00;
					rsp->attr_value.value[10] = 0x00;
					rsp->attr_value.value[11] = 0x00;
			 
					//datalen
					rsp->attr_value.value[12] = 0x01;
					rsp->attr_value.value[13] = 0x00;
					rsp->attr_value.value[14] = 0x00;
					rsp->attr_value.value[15] = 0x00;
			 
					//data
					if(get_work_stat() == 1)
					rsp->attr_value.value[16] = 0x00;
					else
					rsp->attr_value.value[16] = 0x01;	
					//rsp->attr_value.value[17] = 0x00;
					//rsp->attr_value.value[18] = 0x00;
					//rsp->attr_value.value[19] = 0x00;
					//rsp->attr_value.value[20] = 0x11;
			 
					printf("response for cmd:SMP_CMD_SET_WORK_STATE\n");
			 
					/*
					if(rsp->attr_value.len <= 20)
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);
					else{
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								20, rsp->attr_value.value, false);
					
					vTaskDelay(pdMS_TO_TICKS(200));
					
					esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len - 20, rsp->attr_value.value + 20, false);
					
					}*/
					
					 esp_ble_gatts_send_indicate(zt_gatt_if, zt_param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								rsp->attr_value.len, rsp->attr_value.value, false);


	
	
			 break;
			 
			
		default:
			 {
			 


			 break;

			 }
	
	   }
	//rev_write = false;

	rev_write_cmd = 0xFFFF;
	
 	}
vTaskDelay(pdMS_TO_TICKS(1000));
}
}


static void Bt_write_event_rcv(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
   uint16_t SessionID  = param->write.value[1]<<8 | param->write.value[0];
   uint16_t cmd  = param->write.value[3]<<8 | param->write.value[2];
   esp_gatt_rsp_t *rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
   int64_t time_msec;
   esp_gatt_status_t status = ESP_GATT_OK;

   printf("Bt_write_event_rcv cmd = %d\n", cmd);
   printf("param->write.need_rsp = %d param->write.is_prep = %d\n",param->write.need_rsp,param->write.is_prep); 
   switch(cmd){
   	
     case SMP_CMD_GET_DEVICE_MODEL:

    //printf("param->write.need_rsp = %d param->write.is_prep = %d\n",param->write.need_rsp,param->write.is_prep);        
    
         break;


	 case SMP_CMD_GET_WORKING_TIME:

         
         break;

     case SMP_CMD_REPORT_RECORD_WORK_STATE:

	    

         break;

	 case SMP_CMD_SET_RECORD_WORKING_TIME:

        

         break;

	 case SMP_CMD_SET_WORKING_TIME:


         work_time_start = esp_timer_get_time();

	     work_time =  (param->write.value[20] << 24) | (param->write.value[19] << 16) | (param->write.value[18] << 8)| (param->write.value[17]);

		 work_time = work_time*1000*1000;

		 printf("set work time_usec = %lld work_time_star = %lld\n",work_time,work_time_start);
		 
		 if(get_work_stat() == 0)
	     {
			 
			 //set_work(1);
			 gpio_set_level(GPIO_OUTPUT_HEAT_NO, 1);
		 }

		 if (work_timer_run == false)
		 {
		  ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  work_timer_run = true;
		 }
		 else
		 {
		  ESP_ERROR_CHECK(esp_timer_stop(oneshot_timer));
          ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
		  work_timer_run = true;
		 }
		 
        
         break;

	 case SMP_CMD_HEART:


        
		
         break;

	 case SMP_CMD_SET_DEBUG_OUTPUT:


	 
        
         break;

	 case SMP_CMD_SET_WORK_STATE:

		 
		 
		 
		 if (param->write.value[16] == 0)     // start work
			{
			  set_work(1);
			}
		 
		 
		 else if (param->write.value[16] == 1) //end work
			{
			  set_work(0);
			}
		 
		 
		 else if (param->write.value[16] == 3) //shutdown
			{
			  set_power(0);
			}
		 
		 
         break;

	 case SMP_MSG_DEBUG:


	 
		
         break;

	case SMP_CMD_REPORT_WORK_STATE:

        
         break;

    default:

         break;

   }

   
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

/*
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
          //if(1){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}
*/

/*
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
		esp_gatt_status_t status = ESP_GATT_OK;
		if (param->write.need_rsp){
			//if (param->write.is_prep){
			  if(1){
				if (prepare_write_env->prepare_buf == NULL) {
					prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
					prepare_write_env->prepare_len = 0;
					if (prepare_write_env->prepare_buf == NULL) {
						ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
						status = ESP_GATT_NO_RESOURCES;
					}
				} else {
					if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
						status = ESP_GATT_INVALID_OFFSET;
					} else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
						status = ESP_GATT_INVALID_ATTR_LEN;
					}
				}
	
				esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
				gatt_rsp->attr_value.len = 4;//param->write.len; 
				gatt_rsp->attr_value.handle = param->write.handle;
				gatt_rsp->attr_value.offset = param->write.offset;
				gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
				//memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
				
				gatt_rsp->attr_value.value[0] = 0xAA;
				gatt_rsp->attr_value.value[1] = 0xBB;
		 
				//cmd
				gatt_rsp->attr_value.value[2] = 0xCC;
				gatt_rsp->attr_value.value[3] = 0xDD;

				ESP_LOGD(GATTS_TAG, "ZT Send response \n");
				
				esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
				if (response_err != ESP_OK){
				   ESP_LOGE(GATTS_TAG, "Send response error\n");
				}
				free(gatt_rsp);
				if (status != ESP_GATT_OK){
					return;
				}
				memcpy(prepare_write_env->prepare_buf + param->write.offset,
					   param->write.value,
					   param->write.len);
				prepare_write_env->prepare_len += param->write.len;
	
			}else{
				esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
			}
		}
}
*/
#if 1
	void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
			esp_gatt_status_t status = ESP_GATT_OK;
			printf("param->write.need_rsp = %d param->write.is_prep = %d\n",param->write.need_rsp,param->write.is_prep);  
			if (param->write.need_rsp){
				//if (param->write.is_prep){
				  if(1){
			esp_gatt_rsp_t rsp;
			memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
			rsp.attr_value.handle = param->write.handle;
			rsp.attr_value.len = 4;
			/*
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			*/
			rsp.attr_value.value[0] = 0x35;
			rsp.attr_value.value[1] = 0x36;
			rsp.attr_value.value[2] = 0x37;
			rsp.attr_value.value[3] = 0x38;
			//vTaskDelay(pdMS_TO_TICKS(2000));
			
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
										ESP_GATT_OK, &rsp);
										
			/*
			uint8_t notify_data[15];
			for (int i = 0; i < sizeof(notify_data); ++i)
			{
				notify_data[i] = i%0xff + 10;
			}
			//the size of notify_data[] need less than MTU size
			esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
									sizeof(notify_data), notify_data, false);
			
            */
			//vTaskDelay(pdMS_TO_TICKS(2000));
			
		    ESP_LOGE(GATTS_TAG, "ZT Send response \n");
				}else{
					esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
				}
			}
	}
#endif
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {


    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		#if 1
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
		/*
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
		*/
		rsp.attr_value.value[0] = 0x31;
        rsp.attr_value.value[1] = 0x32;
        rsp.attr_value.value[2] = 0x33;
        rsp.attr_value.value[3] = 0x34;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
		#endif
		
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);

	    if(Bt_notify_on == true)
			{
		    rev_write_cmd  = param->write.value[3]<<8 | param->write.value[2];
            rev_write = true;
			}
		
	    zt_gatt_if = gatts_if;
	    zt_param = param;
				
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
		    ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, a descr_handle: %d", gl_profile_tab[PROFILE_A_APP_ID].descr_handle);


					
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                //if(1){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");


                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
						Bt_notify_on = true;
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }

            }
			//Bt_write_event_rcv(gatts_if,param);
        }
		//Bt_write_event_rcv(gatts_if,param);
		
            printf("Bt_write_event_rcv start\n");
			Bt_write_event_rcv(gatts_if,&a_prepare_write_env,param);
			printf("Bt_write_event_rcv end\n");
		//if(Bt_responce_on == true)
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
		
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        gpio_set_level(GPIO_OUTPUT_LED_G, 0);
        gpio_set_level(GPIO_OUTPUT_LED_R, 0);
        for(int i = 0; i < 5; i++)
		{
		gpio_set_level(GPIO_OUTPUT_LED_B, 1);
		vTaskDelay(500 / portTICK_RATE_MS);
		gpio_set_level(GPIO_OUTPUT_LED_B, 0);
		vTaskDelay(500 / portTICK_RATE_MS);
		ESP_LOGI(GATTS_TAG, "BT connect %d\n", i);
		}
		gpio_set_level(GPIO_OUTPUT_LED_B, 1);
		ESP_LOGI(GATTS_TAG, "BT connected \n");

        esp_ble_gap_update_conn_params(&conn_params);
        Bt_connect = true;
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		gpio_set_level(GPIO_OUTPUT_LED_B, 0);
	    gpio_set_level(GPIO_OUTPUT_LED_G, 1);
        esp_ble_gap_start_advertising(&adv_params);
		Bt_connect = false;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_B);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
		    ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, b descr_handle: %d", gl_profile_tab[PROFILE_A_APP_ID].descr_handle);
            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown value");
                }

            }
        }
		Bt_write_event_rcv(gatts_if,&b_prepare_write_env,param);
        //example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
		/*
        gpio_set_level(GPIO_OUTPUT_LED_G, 0);
        gpio_set_level(GPIO_OUTPUT_LED_R, 0);
        for(int i = 0; i < 5; i++)
		{
		gpio_set_level(GPIO_OUTPUT_LED_B, 1);
		vTaskDelay(1000 / portTICK_RATE_MS);
		gpio_set_level(GPIO_OUTPUT_LED_B, 0);
		ESP_LOGI(GATTS_TAG, "BT connect %d\n", i);
		}
		gpio_set_level(GPIO_OUTPUT_LED_B, 1);
		ESP_LOGI(GATTS_TAG, "BT connected \n");
        */
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
    break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main()
{
    esp_err_t ret;

    printf("ZT FAR_IR init VSRSION:V.%d.%d\n", VERSION_MARJOR,VERSION_MINOR);

	ZT_Bt_init();

	//Bt_open = true;
	
	// GPIO CONTROL
	
	gpio_config_t io_conf;
	//disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_VDET_KEY, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_VDET_KEY, gpio_isr_handler, (void*) GPIO_INPUT_VDET_KEY);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_VDET_KEY);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_VDET_KEY, gpio_isr_handler, (void*) GPIO_INPUT_VDET_KEY);

	gpio_set_level(GPIO_OUTPUT_PWR_CTL, 1);
	gpio_set_level(GPIO_OUTPUT_HEAT_NO, 1);
	//gpio_set_level(GPIO_OUTPUT_LED_G, 1);
	//vTaskDelay(1000 / portTICK_RATE_MS);
	//gpio_set_level(GPIO_OUTPUT_LED_B, 0);
	//gpio_set_level(GPIO_OUTPUT_LED_B, 1);
    HEAT_RELAY_STAT = 1;

    //ADC Start
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);


	//worrktimer start
	//esp_timer_handle_t periodic_timer;

    //esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_create(&work_timer_args, &oneshot_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, work_time));
	work_timer_run = true;

	work_time_start = esp_timer_get_time();

	gpio_set_level(GPIO_OUTPUT_LED_G, 1);

	//xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);

	xTaskCreate(bt_task_notify, "bt_task_notify", 2048, NULL, 10, NULL);
	
    int cnt = 0;
    while(1) {
        printf("cnt: %d\n", cnt++);

	    //Set LED
	    /*
        vTaskDelay(500 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_LED_R, cnt % 2);
        vTaskDelay(500 / portTICK_RATE_MS);
		gpio_set_level(GPIO_OUTPUT_LED_G, cnt % 2);
		vTaskDelay(500 / portTICK_RATE_MS);
		gpio_set_level(GPIO_OUTPUT_LED_B, cnt % 2);
		*/
        //gpio_set_level(GPIO_OUTPUT_LED_G, cnt % 2);
		//vTaskDelay(500 / portTICK_RATE_MS);
		//printf("get led R: %d,get led G:%d,get led B:%d\n",gpio_get_level(GPIO_OUTPUT_LED_R),gpio_get_level(GPIO_OUTPUT_LED_R),gpio_get_level(GPIO_OUTPUT_LED_B));

        //ADC Check IO26
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		//voltage = (voltage*3300)/1000;
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);

		#if 1
		if(voltage < 395 && work_overtemp_stop == false)
		{
			gpio_set_level(GPIO_OUTPUT_HEAT_NO, 0);
			gpio_set_level(GPIO_OUTPUT_LED_G, 0);
			gpio_set_level(GPIO_OUTPUT_LED_B, 0);
			gpio_set_level(GPIO_OUTPUT_LED_R, 1);
			
			work_overtemp_stop = true;
			
			HEAT_RELAY_STAT = 0;
			ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, work_overtemp_reopen_time));
			
			ESP_LOGE(GATTS_TAG, "over temprature turn off relay:200 HEAT_RELAY_STAT = %d\n",HEAT_RELAY_STAT);
			
		}
		else
		{
			if(work_overtemp_stop == true && work_overtemp_reopen == true)
			{
			gpio_set_level(GPIO_OUTPUT_HEAT_NO, 1);
			if(Bt_connect == true)
			{
				gpio_set_level(GPIO_OUTPUT_LED_R, 0);
				gpio_set_level(GPIO_OUTPUT_LED_G, 0);
			    gpio_set_level(GPIO_OUTPUT_LED_B, 1);
			
			}
			else
			{
				gpio_set_level(GPIO_OUTPUT_LED_R, 0);
				gpio_set_level(GPIO_OUTPUT_LED_B, 0);
				gpio_set_level(GPIO_OUTPUT_LED_G, 1);
			}
			work_overtemp_reopen = false;
			work_overtemp_stop = false;
			
			HEAT_RELAY_STAT = 1;
			ESP_LOGI(GATTS_TAG, "over temprature time over reopen relay: HEAT_RELAY_STAT =%d\n",HEAT_RELAY_STAT);
			}
		}
		#endif
		
        vTaskDelay(pdMS_TO_TICKS(1000));
		//printf("get heat io value: %d,power control io value:%d\n",gpio_get_level(GPIO_OUTPUT_HEAT_NO),gpio_get_level(GPIO_OUTPUT_PWR_CTL));
		/*
		for (int i = 0; i < 5; ++i) {
        ESP_ERROR_CHECK(esp_timer_dump(stdout));
        //usleep(2000000);
        vTaskDelay(2000 / portTICK_RATE_MS);
        }*/
    }
	
    
}

static void periodic_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
	work_overtemp_reopen = true;
	ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_LOGI(GATTS_TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
}

static void work_timer_callback(void* arg)
{
    work_time_end = esp_timer_get_time();
    ESP_LOGI(GATTS_TAG, "work timer called, work_time_end : %lld us, time since start: %lld us", work_time_end,work_time_end-work_time_start);
	
	gpio_set_level(GPIO_OUTPUT_HEAT_NO, 0);

	HEAT_RELAY_STAT = 0;

	work_timer_run = false;
	
	if(Bt_connect == false || Bt_open == false)
	{
        offline_work_count ++;

		offline_work_time[offline_work_count] = work_time_end - work_time_start;

		

	}
}

static void ZT_Bt_init(void)
{

   esp_err_t ret;

    printf("ZT FAR_IR bt init\n");
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

	/*
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }*/
    
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    Bt_open = true;
}


static void ZT_Bt_deinit(void)
{


   esp_err_t ret;

    printf("ZT FAR_IR bt deinit\n");

    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s disable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

	ret = esp_bluedroid_deinit();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s deinit bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s disable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

}

