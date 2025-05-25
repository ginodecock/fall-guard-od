 /**
 ******************************************************************************
 * @file    app.c
 * @author  G-DC
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 G-DC
 * All rights reserved.
 *
 * This software comes is provided AS-IS.
 *
 ******************************************************************************
 */
#include "cmw_camera.h"
#include "app.h"
#include "nxd_dhcp_client.h"
#include "nx_ip.h"
#include "nx_api.h"
#include <stdint.h>

#include "app_cam.h"
#include "app_config.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "ll_aton_runtime.h"
#include "cmw_camera.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6xx_hal.h"
#include "tx_api.h"
#include "utils.h"
#include "crop_img.h"
#include <string.h> // For memcpy
/* Includes ------------------------------------------------------------------*/
//#include "app_netxduo.h"
//UINT MX_NetXDuo_Init(VOID *memory_ptr);
/* Private includes ----------------------------------------------------------*/
#include "nx_user.h"
/* USER CODE BEGIN Includes */
//#include  MOSQUITTO_CERT_FILE
#include "tx_api.h"
#include "nxd_mqtt_client.h"
#include "nxd_sntp_client.h"
#include "nxd_dns.h"
#include "nx_stm32_eth_driver.h"
//#include "app_azure_rtos.h"
#include "stm32n6xx.h"
#include "app_threadx.h"
BSP_LCD_LayerConfig_t LayerConfig = {0};
/* Private variables ---------------------------------------------------------*/
#define USE_STATIC_ALLOCATION                    1

#define TX_APP_MEM_POOL_SIZE                     1024

#define NX_APP_MEM_POOL_SIZE                     50*1024

#if (USE_STATIC_ALLOCATION == 1)
/* USER CODE BEGIN TX_Pool_Buffer */
/* USER CODE END TX_Pool_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE] __ALIGN_END;
static TX_BYTE_POOL tx_app_byte_pool;

/* USER CODE BEGIN NX_Pool_Buffer */
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = ".NetXPoolSection"
#else /* GNU and AC6 compilers */
__attribute__((section(".NetXPoolSection")))
#endif
/* USER CODE END NX_Pool_Buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static UCHAR nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE] __ALIGN_END;
static TX_BYTE_POOL nx_app_byte_pool;

#endif
static void Display_WelcomeScreen(void);
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
TX_SEMAPHORE   DHCPSemaphore;
NX_DHCP        DHCPClient;

ULONG          IpAddress;
ULONG          NetMask;
/* USER CODE BEGIN PV */
//extern RNG_HandleTypeDef hrng;

TX_THREAD AppMQTTClientThread;
TX_THREAD AppMQTTClientThreadLog;
TX_THREAD AppSNTPThread;
TX_THREAD AppLinkThread;

TX_QUEUE  MsgQueueOne;

NXD_MQTT_CLIENT MqttClient;
NX_SNTP_CLIENT  SntpClient;
static NX_DNS   DnsClient;
uint8_t MACAddr[6];
RTC_HandleTypeDef hrtc;
uint8_t prev_nb_person;
uint8_t prev_nb_detect;
ULONG prev_timestamp;
int prev_state_detect;
// Shared resources between threads
#define MEASUREMENT_QUEUE_DEPTH  20
#define MEASUREMENT_QUEUE_MSG_SIZE sizeof(sensor_data_t)

/* Define a structure for your sensor data */
typedef struct {
    float nb_detect;
    ULONG timestamp;
    int state_detect;
} sensor_data_t;

/* Create a message queue */
TX_QUEUE measurement_queue;
UCHAR measurement_queue_buffer[MEASUREMENT_QUEUE_DEPTH * MEASUREMENT_QUEUE_MSG_SIZE];


#define QUEUE_DEPTH 10
#define QUEUE_MSG_SIZE 256

TX_EVENT_FLAGS_GROUP     SntpFlags;

ULONG mqtt_client_stack[MQTT_CLIENT_STACK_SIZE];

TX_EVENT_FLAGS_GROUP mqtt_app_flag;

/* Declare buffers to hold message and topic. */
static char message[NXD_MQTT_MAX_MESSAGE_LENGTH];
static UCHAR message_buffer[NXD_MQTT_MAX_MESSAGE_LENGTH];
static UCHAR topic_buffer[NXD_MQTT_MAX_TOPIC_NAME_LENGTH];
/* TLS buffers and certificate containers. */
//extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers;
/* calculated with nx_secure_tls_metadata_size_calculate */
//static CHAR crypto_metadata_client[11600];
/* Define the TLS packet reassembly buffer. */
UCHAR tls_packet_buffer[4000];
ULONG current_time;
/* USER CODE END PV */
static void Display_NetworkOutput(od_pp_out_t *p_postprocess, uint32_t inference_ms);

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);
/* USER CODE BEGIN PFP */
static VOID App_MQTT_Client_Thread_Entry(ULONG thread_input);
static VOID App_SNTP_Thread_Entry(ULONG thread_input);
static VOID App_Link_Thread_Entry(ULONG thread_input);

static VOID time_update_callback(NX_SNTP_TIME_MESSAGE *time_update_ptr, NX_SNTP_TIME *local_time);
//static ULONG nx_secure_tls_session_time_function(void);
static UINT dns_create(NX_DNS *dns_ptr);

/*static UINT tls_setup_callback(NXD_MQTT_CLIENT *client_pt,
                        NX_SECURE_TLS_SESSION *TLS_session_ptr,
                        NX_SECURE_X509_CERT *certificate_ptr,
                        NX_SECURE_X509_CERT *trusted_certificate_ptr);
*/
/* USER CODE END PFP */

static void SetRtcFromEpoch(uint32_t epoch);
static uint32_t GetRtcEpoch();


#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

/*#define LCD_FG_WIDTH LCD_BG_WIDTH
#define LCD_FG_HEIGHT LCD_BG_HEIGHT*/
volatile int32_t cameraFrameReceived;
#define NUMBER_COLORS 10
#define BQUEUE_MAX_BUFFERS 3
#define CPU_LOAD_HISTORY_DEPTH 8

#define DISPLAY_BUFFER_NB (DISPLAY_DELAY + 2)

/* Align so we are sure nn_output_buffers[0] and nn_output_buffers[1] are aligned on 32 bytes */
#define NN_BUFFER_OUT_SIZE_ALIGN ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)
typedef struct NXD_MQTT_MESSAGE_STRUCT {
    UCHAR *topic;       /* Pointer to message topic */
    ULONG topic_length; /* Topic length in bytes */
    UCHAR *payload;     /* Pointer to message payload */
    ULONG payload_length; /* Payload length in bytes */
    UCHAR qos_level;    /* QoS level of message */
    UCHAR retain;       /* Retain flag */
} NXD_MQTT_MESSAGE;

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

/* Globals */
DECLARE_CLASSES_TABLE;
/* Lcd Background area */
static Rectangle_TypeDef lcd_bg_area = {
  .X0 = 80,
  .Y0 = 0,
  .XSize = 0,
  .YSize = 0,
};
/* Lcd Foreground area */
static Rectangle_TypeDef lcd_fg_area = {
  .X0 = 0,
  .Y0 = 0,
  .XSize = 800,
  .YSize = 480,
};
static const uint32_t colors[NUMBER_COLORS] = {
    UTIL_LCD_COLOR_CYAN,
    UTIL_LCD_COLOR_GREEN,
    UTIL_LCD_COLOR_RED,
    UTIL_LCD_COLOR_MAGENTA,
    UTIL_LCD_COLOR_YELLOW,
    UTIL_LCD_COLOR_GRAY,
    UTIL_LCD_COLOR_BLACK,
    UTIL_LCD_COLOR_BROWN,
    UTIL_LCD_COLOR_BLUE,
    UTIL_LCD_COLOR_ORANGE
};

#define ALIGN_TO_16(value) (((value) + 15) & ~15)

/* for models not multiple of 16; needs a working buffer */
#if (NN_WIDTH * NN_BPP) != ALIGN_TO_16(NN_WIDTH * NN_BPP)
#define DCMIPP_OUT_NN_LEN (ALIGN_TO_16(NN_WIDTH * NN_BPP) * NN_HEIGHT)
#define DCMIPP_OUT_NN_BUFF_LEN (DCMIPP_OUT_NN_LEN + 32 - DCMIPP_OUT_NN_LEN%32)

__attribute__ ((aligned (32)))
uint8_t dcmipp_out_nn[DCMIPP_OUT_NN_BUFF_LEN];
#else
uint8_t *dcmipp_out_nn;
#endif
/* Lcd Background Buffer */
__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t lcd_bg_buffer[800 * 480 * 2];
/* Lcd Foreground Buffer */
__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT * 2];
static int lcd_fg_buffer_rd_idx;
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);
volatile int32_t cameraFrameReceived;
static TX_THREAD nn_thread;
static uint8_t nn_tread_stack[4096];

static void Display_NetworkOutput(od_pp_out_t *p_postprocess, uint32_t inference_ms)
{
  sensor_data_t data;
  od_pp_outBuffer_t *rois = p_postprocess->pOutBuff;
  uint32_t nb_rois = p_postprocess->nb_detect;
  int ret;

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
  assert(ret == HAL_OK);

  /* Draw bounding boxes */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */
  for (int32_t i = 0; i < nb_rois; i++)
  {
    uint32_t x0 = (uint32_t) ((rois[i].x_center - rois[i].width / 2) * ((float32_t) lcd_bg_area.XSize)) + lcd_bg_area.X0;
    uint32_t y0 = (uint32_t) ((rois[i].y_center - rois[i].height / 2) * ((float32_t) lcd_bg_area.YSize));
    uint32_t width = (uint32_t) (rois[i].width * ((float32_t) lcd_bg_area.XSize));
    uint32_t height = (uint32_t) (rois[i].height * ((float32_t) lcd_bg_area.YSize));
    /* Draw boxes without going outside of the image to avoid clearing the text area to clear the boxes */
    x0 = x0 < lcd_bg_area.X0 + lcd_bg_area.XSize ? x0 : lcd_bg_area.X0 + lcd_bg_area.XSize - 1;
    y0 = y0 < lcd_bg_area.Y0 + lcd_bg_area.YSize ? y0 : lcd_bg_area.Y0 + lcd_bg_area.YSize  - 1;
    width = ((x0 + width) < lcd_bg_area.X0 + lcd_bg_area.XSize) ? width : (lcd_bg_area.X0 + lcd_bg_area.XSize - x0 - 1);
    height = ((y0 + height) < lcd_bg_area.Y0 + lcd_bg_area.YSize) ? height : (lcd_bg_area.Y0 + lcd_bg_area.YSize - y0 - 1);
    UTIL_LCD_DrawRect(x0, y0, width, height, colors[rois[i].class_index % NUMBER_COLORS]);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, classes_table[rois[i].class_index]);
    UTIL_LCDEx_PrintfAt(-x0-width, y0, RIGHT_MODE, "%.0f%%", rois[i].conf*100.0f);
    //printf("color = %lu",rois[i].class_index);
    if (rois[i].class_index == 2 ){
    	data.nb_detect = nb_rois;
    	data.timestamp = GetRtcEpoch();
    	data.state_detect = 13; //Fall detected!
    	/* Send to MQTT thread */
    	if (prev_state_detect != 13){
    	   	 tx_queue_send(&measurement_queue, &data, TX_NO_WAIT);
    	   	 prev_state_detect = 13;
    	}
    }
  }
  if ((GetRtcEpoch() - prev_timestamp) > 3)
    {
  	  prev_timestamp = GetRtcEpoch();
        if (nb_rois != prev_nb_person){
  	     printf("persons detected = %lu\n\r",nb_rois);
           prev_nb_person = nb_rois;
  	     /* Allocate the memory for MQTT client thread   */
  	     data.nb_detect = prev_nb_person;
  	     data.timestamp = GetRtcEpoch();
  	     data.state_detect = 1; //Normal person state
  	     prev_state_detect = 1;

  	     /* Send to MQTT thread */
  	     tx_queue_send(&measurement_queue, &data, TX_NO_WAIT);
   	  }
    }

  UTIL_LCD_SetBackColor(0x40000000);
  UTIL_LCDEx_PrintfAt(0, LINE(2), CENTER_MODE, "Objects %u", nb_rois);
  UTIL_LCDEx_PrintfAt(0, LINE(20), CENTER_MODE, "Inference: %ums", inference_ms);
  UTIL_LCD_SetBackColor(0);

  Display_WelcomeScreen();

  SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_FRAMEBUFFER_SIZE);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
  assert(ret == HAL_OK);
  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

static void LCD_init()
{
  printf("Display init \n\r");

  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);

  /* Preview layer Init */
  LayerConfig.X0          = lcd_bg_area.X0;
  LayerConfig.Y0          = lcd_bg_area.Y0;
  LayerConfig.X1          = lcd_bg_area.X0 + lcd_bg_area.XSize;
  LayerConfig.Y1          = lcd_bg_area.Y0 + lcd_bg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_RGB565;
  LayerConfig.Address     = (uint32_t) lcd_bg_buffer;

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &LayerConfig);

  LayerConfig.X0 = lcd_fg_area.X0;
  LayerConfig.Y0 = lcd_fg_area.Y0;
  LayerConfig.X1 = lcd_fg_area.X0 + lcd_fg_area.XSize;
  LayerConfig.Y1 = lcd_fg_area.Y0 + lcd_fg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_ARGB4444;
  LayerConfig.Address = (uint32_t) lcd_fg_buffer; /* External XSPI1 PSRAM */

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &LayerConfig);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(LTDC_LAYER_2);
  UTIL_LCD_Clear(0x00000000);
  UTIL_LCD_SetFont(&Font20);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
}

static void nn_thread_fct(ULONG arg)
{
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
  yolov2_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
  yolov5_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF || POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UI
  yolov8_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_ST_SSD_UF
  ssd_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_ST_YOLOX_UF
  st_yolox_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_ISEG_YOLO_V8_UI
  yolov8_pp_static_param_t pp_params;
#else
    #error "PostProcessing type not supported"
#endif
  od_pp_out_t pp_output;
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_Default();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_Default();
  uint8_t *nn_in;
  nn_in = (uint8_t *) LL_Buffer_addr_start(&nn_in_info[0]);
  float32_t *nn_out[5];
  int32_t nn_out_len[5];
  int number_output = 3;//0;

  /* Count number of outputs */
  while (nn_out_info[number_output].name != NULL)
  {
    number_output++;
  }
  assert(number_output <= 5);

  for (int i = 0; i < number_output; i++)
  {
    nn_out[i] = (float32_t *) LL_Buffer_addr_start(&nn_out_info[i]);
    nn_out_len[i] = LL_Buffer_len(&nn_out_info[i]);
  }

  uint32_t nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  uint32_t pitch_nn = 0;

  UNUSED(nn_in_len);
  app_postprocess_init(&pp_params);

  /*** App Loop ***************************************************************/
  CAM_Init(&lcd_bg_area.XSize, &lcd_bg_area.YSize, &pitch_nn);
  LCD_init();

  /* Start LCD Display camera pipe stream */
  printf("start pipe\n\r");
  CAM_DisplayPipe_Start(lcd_bg_buffer, CMW_MODE_CONTINUOUS);
  printf("pipe started\n\r");
  while (1)
  {
	    CAM_IspUpdate();
	    if (pitch_nn != (NN_WIDTH * NN_BPP))
	    {
	      /* Start NN camera single capture Snapshot */
	      CAM_NNPipe_Start(dcmipp_out_nn, CMW_MODE_SNAPSHOT);
	    }
	    else
	    {
	      /* Start NN camera single capture Snapshot */
	      CAM_NNPipe_Start(nn_in, CMW_MODE_SNAPSHOT);
	   }

	    while (cameraFrameReceived == 0) {};
	    cameraFrameReceived = 0;

	    uint32_t ts[2] = { 0 };

	    if (pitch_nn != (NN_WIDTH * NN_BPP))
	    {
	      SCB_InvalidateDCache_by_Addr(dcmipp_out_nn, sizeof(dcmipp_out_nn));
	      img_crop(dcmipp_out_nn, nn_in, pitch_nn, NN_WIDTH, NN_HEIGHT, NN_BPP);
	      SCB_CleanInvalidateDCache_by_Addr(nn_in, nn_in_len);
	    }

	    ts[0] = HAL_GetTick();
	    /* run ATON inference */
	    LL_ATON_RT_Main(&NN_Instance_Default);
	    ts[1] = HAL_GetTick();

	    int32_t ret = app_postprocess_run((void **) nn_out, number_output, &pp_output, &pp_params);
	    assert(ret == 0);

	    Display_NetworkOutput(&pp_output, ts[1] - ts[0]);
	    /* Discard nn_out region (used by pp_input and pp_outputs variables) to avoid Dcache evictions during nn inference */
	    for (int i = 0; i < number_output; i++)
	    {
	      float32_t *tmp = nn_out[i];
	      SCB_InvalidateDCache_by_Addr(tmp, nn_out_len[i]);
	    }


  }
}
void app_run()
{
  const UINT nn_priority = TX_MAX_PRIORITIES / 2 - 1;
  const ULONG time_slice = 10;
  int ret;

  printf("\n\rInit application\n\r");
  ret = tx_thread_create(&nn_thread, "nn", nn_thread_fct, 0, nn_tread_stack,
                         sizeof(nn_tread_stack), nn_priority, nn_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
  UINT status = TX_SUCCESS;
  VOID *memory_ptr;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    Error_Handler();
  }
  else
  {
    memory_ptr = (VOID *)&tx_app_byte_pool;
    status = App_ThreadX_Init(memory_ptr);
    if (status != TX_SUCCESS)
    {
      Error_Handler();
    }
  }

  if (tx_byte_pool_create(&nx_app_byte_pool, "Nx App memory pool", nx_byte_pool_buffer, NX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    Error_Handler();
  }
  else
  {
    memory_ptr = (VOID *)&nx_app_byte_pool;
    status = MX_NetXDuo_Init(memory_ptr);
    if (status != NX_SUCCESS)
    {
      Error_Handler();
    }
  }
}

UINT string_to_ip(const char *ip_string, ULONG *ip_address)
{
    UINT byte1, byte2, byte3, byte4;

    if (sscanf(ip_string, "%u.%u.%u.%u", &byte1, &byte2, &byte3, &byte4) == 4)
    {
        *ip_address = ((byte1 & 0xFF) << 24) | ((byte2 & 0xFF) << 16) |
                      ((byte3 & 0xFF) << 8) | (byte4 & 0xFF);
        return NX_SUCCESS;
    }
    return NX_NOT_SUCCESSFUL;
}


/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer;
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }
  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }
  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }
  ret = nx_dhcp_create(&DHCPClient, &NetXDuoEthIpInstance, "DHCP Client");

  if (ret != NX_SUCCESS)
  {
    return NX_DHCP_ERROR;
  }
  tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }
  printf("Nx_MQTT_Client application started..\n\r");
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, SNTP_CLIENT_THREAD_MEMORY, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  ret = tx_thread_create(&AppSNTPThread, "App SNTP Thread", App_SNTP_Thread_Entry, 0, pointer, SNTP_CLIENT_THREAD_MEMORY,
                         SNTP_PRIORITY, SNTP_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  ret = tx_event_flags_create(&SntpFlags, "SNTP event flags");
  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, THREAD_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  ret = tx_thread_create(&AppMQTTClientThread, "App MQTT Thread", App_MQTT_Client_Thread_Entry, 0, pointer, THREAD_MEMORY_SIZE,
                         MQTT_PRIORITY, MQTT_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,2 *  DEFAULT_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  ret = tx_thread_create(&AppLinkThread, "App Link Thread", App_Link_Thread_Entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE,
                         LINK_PRIORITY, LINK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }
  /* Allocate the MsgQueueOne.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, APP_QUEUE_SIZE*sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }
  /* Create the MsgQueueOne shared by MsgSenderThreadOne and MsgReceiverThread */
  if (tx_queue_create(&MsgQueueOne, "Message Queue One",TX_1_ULONG, pointer, APP_QUEUE_SIZE*sizeof(ULONG)) != TX_SUCCESS)
  {
    ret = TX_QUEUE_ERROR;
  }
  return ret;
}

/**
* @brief  ip address change callback.
* @param ip_instance: NX_IP instance
* @param ptr: user data
* @retval none
*/
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
  if (nx_ip_address_get(&NetXDuoEthIpInstance, &IpAddress, &NetMask) != NX_SUCCESS)
  {
    Error_Handler();
  }
  if(IpAddress != NULL_ADDRESS)
  {
    tx_semaphore_put(&DHCPSemaphore);
  }
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID nx_app_thread_entry (ULONG thread_input)
{
  UINT ret = NX_SUCCESS;
  ret = dns_create(&DnsClient);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
  ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
  ret = nx_dhcp_start(&DHCPClient);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  printf("Looking for DHCP server ..\n\r");
  if(tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
	  printf("Looking for DHCP server ..error\n\r");
    Error_Handler();
  }
  PRINT_IP_ADDRESS(IpAddress);
  tx_thread_resume(&AppSNTPThread);
  tx_thread_relinquish();
}
/**
* @brief  DNS Create Function.
* @param dns_ptr
* @retval ret
*/
UINT dns_create(NX_DNS *dns_ptr)
{
  UINT ret = NX_SUCCESS;

  /* Create a DNS instance for the Client */
  ret = nx_dns_create(dns_ptr, &NetXDuoEthIpInstance, (UCHAR *)"DNS Client");
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
  /* Initialize DNS instance with a dummy server */
  ret = nx_dns_server_add(dns_ptr, USER_DNS_ADDRESS);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  return ret;
}

/* Declare the disconnect notify function. */
static VOID my_disconnect_func(NXD_MQTT_CLIENT *client_ptr)
{
  NX_PARAMETER_NOT_USED(client_ptr);
  printf("client disconnected from broker < %s >.\n\r", MQTT_BROKER_NAME);
}

/* Declare the notify function. */
static VOID my_notify_func(NXD_MQTT_CLIENT* client_ptr, UINT number_of_messages)
{
  NX_PARAMETER_NOT_USED(client_ptr);
  NX_PARAMETER_NOT_USED(number_of_messages);
  tx_event_flags_set(&mqtt_app_flag, DEMO_MESSAGE_EVENT, TX_OR);
  return;
}

/* Function (set by user) to call when TLS needs the current time. */
/*ULONG nx_secure_tls_session_time_function(void)
{
  return (current_time);
}*/

/* Callback to setup TLS parameters for secure MQTT connection. */
/*static UINT tls_setup_callback(NXD_MQTT_CLIENT *client_pt,
                        NX_SECURE_TLS_SESSION *TLS_session_ptr,
                        NX_SECURE_X509_CERT *certificate_ptr,
                        NX_SECURE_X509_CERT *trusted_certificate_ptr)
{
  UINT ret = NX_SUCCESS;
  NX_PARAMETER_NOT_USED(client_pt);
*/
  /* Initialize TLS module */
/*  nx_secure_tls_initialize();
*/
  /* Create a TLS session */
/*  ret = nx_secure_tls_session_create(TLS_session_ptr, &nx_crypto_tls_ciphers,
                                     crypto_metadata_client, sizeof(crypto_metadata_client));
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
*/
  /* Need to allocate space for the certificate coming in from the broker. */
/*  memset((certificate_ptr), 0, sizeof(NX_SECURE_X509_CERT));

  ret = nx_secure_tls_session_time_function_set(TLS_session_ptr, nx_secure_tls_session_time_function);

  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
*/
  /* Allocate space for packet reassembly. */
/*  ret = nx_secure_tls_session_packet_buffer_set(TLS_session_ptr, tls_packet_buffer,
                                                sizeof(tls_packet_buffer));
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
*/
  /* allocate space for the certificate coming in from the remote host */
/*  ret = nx_secure_tls_remote_certificate_allocate(TLS_session_ptr, certificate_ptr,
                                                  tls_packet_buffer, sizeof(tls_packet_buffer));
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
*/
  /* initialize Certificate to verify incoming server certificates. */
/*  ret = nx_secure_x509_certificate_initialize(trusted_certificate_ptr, (UCHAR*)mosquitto_org_der,
                                              mosquitto_org_der_len, NX_NULL, 0, NULL, 0,
                                              NX_SECURE_X509_KEY_TYPE_NONE);
  if (ret != NX_SUCCESS)
  {
    printf("Certificate issue..\nPlease make sure that your X509_certificate is valid. \n\r");
    Error_Handler();
  }
*/
  /* Add a CA Certificate to our trusted store */
/*
  ret = nx_secure_tls_trusted_certificate_add(TLS_session_ptr, trusted_certificate_ptr);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  return ret;
}
*/
/* This callback defines a handler for notifying SNTP time update event.  */
static VOID time_update_callback(NX_SNTP_TIME_MESSAGE *time_update_ptr, NX_SNTP_TIME *local_time)
{
  NX_PARAMETER_NOT_USED(time_update_ptr);
  NX_PARAMETER_NOT_USED(local_time);

  tx_event_flags_set(&SntpFlags, SNTP_UPDATE_EVENT, TX_OR);
}

/** @brief  SNTP Client thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID App_SNTP_Thread_Entry(ULONG thread_input)
{
  UINT ret;
  ULONG  fraction;
  ULONG  events = 0;
  UINT   server_status;
  NXD_ADDRESS sntp_server_ip;

  sntp_server_ip.nxd_ip_version = 4;

  /* Look up SNTP Server address. */
  ret = nx_dns_host_by_name_get(&DnsClient, (UCHAR *)SNTP_SERVER_NAME, &sntp_server_ip.nxd_ip_address.v4, DEFAULT_TIMEOUT);

  /* Check for error. */
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the SNTP Client */
  ret =  nx_sntp_client_create(&SntpClient, &NetXDuoEthIpInstance, 0, &NxAppPool, NULL, NULL, NULL);

  /* Check for error. */
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Setup time update callback function. */
  nx_sntp_client_set_time_update_notify(&SntpClient, time_update_callback);

  /* Use the IPv4 service to set up the Client and set the IPv4 SNTP server. */
  ret = nx_sntp_client_initialize_unicast(&SntpClient, sntp_server_ip.nxd_ip_address.v4);

  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Run whichever service the client is configured for. */
  ret = nx_sntp_client_run_unicast(&SntpClient);

  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Wait for a server update event. */
  tx_event_flags_get(&SntpFlags, SNTP_UPDATE_EVENT, TX_OR_CLEAR, &events, PERIODIC_CHECK_INTERVAL);

  if (events == SNTP_UPDATE_EVENT)
  {
    /* Check for valid SNTP server status. */
    ret = nx_sntp_client_receiving_updates(&SntpClient, &server_status);

    if ((ret != NX_SUCCESS) || (server_status == NX_FALSE))
    {
      /* We do not have a valid update. */
      Error_Handler();
    }
    /* We have a valid update.  Get the SNTP Client time.  */
    ret = nx_sntp_client_get_local_time_extended(&SntpClient, &current_time, &fraction, NX_NULL, 0);

    if (ret != NX_SUCCESS)
    {
      Error_Handler();
    }
    /* take off 70 years difference */
    current_time -= EPOCH_TIME_DIFF;
    SetRtcFromEpoch(current_time);

  }
  else
  {
    Error_Handler();
  }

  /* start the MQTT client thread */
  tx_thread_resume(&AppMQTTClientThread);

}
/* Function to check if the client is connected */
UINT mqtt_client_is_connected(NXD_MQTT_CLIENT *client)
{
    return client->nxd_mqtt_client_state == NXD_MQTT_CLIENT_STATE_CONNECTED;
}

/**
* @brief  MQTT Client thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID App_MQTT_Client_Thread_Entry(ULONG thread_input)
{
  UINT ret = NX_SUCCESS;
  NXD_ADDRESS mqtt_server_ip;
  ULONG events;
  int len;
  UINT topic_length, message_length;
  UINT message_count = 0;
  sensor_data_t sensor_data;

  /* Initialize message queue */
  ret = tx_queue_create(&measurement_queue, "Measurement Queue",
                           MEASUREMENT_QUEUE_MSG_SIZE,
                           measurement_queue_buffer,  // Ensure you have a buffer
                           MEASUREMENT_QUEUE_DEPTH * MEASUREMENT_QUEUE_MSG_SIZE);
  if (ret != TX_SUCCESS) Error_Handler();
  mqtt_server_ip.nxd_ip_version = 4;

  /* Look up MQTT Server address. */
  ret = nx_dns_host_by_name_get(&DnsClient, (UCHAR *)MQTT_BROKER_NAME,
                                &mqtt_server_ip.nxd_ip_address.v4, DEFAULT_TIMEOUT);

  /* Check status.  */
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create MQTT client instance. */
  ret = nxd_mqtt_client_create(&MqttClient, "my_client", CLIENT_ID_STRING, STRLEN(CLIENT_ID_STRING),
                               &NetXDuoEthIpInstance, &NxAppPool, (VOID*)mqtt_client_stack, MQTT_CLIENT_STACK_SIZE,
                               MQTT_THREAD_PRIORTY, NX_NULL, 0);

  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* Register the disconnect notification function. */
  nxd_mqtt_client_disconnect_notify_set(&MqttClient, my_disconnect_func);

  /* Set the receive notify function. */
  nxd_mqtt_client_receive_notify_set(&MqttClient, my_notify_func);

  /* Create an MQTT flag */
  ret = tx_event_flags_create(&mqtt_app_flag, "my app event");
  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  //ret = nxd_mqtt_client_secure_connect(&MqttClient, &DefaultNXDAddress, MQTT_PORT, tls_setup_callback,MQTT_KEEP_ALIVE_TIMER, CLEAN_SESSION, NX_WAIT_FOREVER);
  ret = nxd_mqtt_client_connect(&MqttClient, &mqtt_server_ip, 1883,MQTT_KEEP_ALIVE_TIMER, CLEAN_SESSION, NX_WAIT_FOREVER);
  if (ret != NX_SUCCESS)
  {
    printf("\nMQTT client failed to connect to broker < %s >at PORT %d.\n\r",MQTT_BROKER_NAME, MQTT_PORT);
    Error_Handler();
  }
  else
  {
    printf("\nMQTT client connected to broker < %s > at PORT %d :\n\r",MQTT_BROKER_NAME, MQTT_PORT);
  }

  /* Subscribe to the topic with QoS level 1. */
  ret = nxd_mqtt_client_subscribe(&MqttClient, TOPIC_NAME, STRLEN(TOPIC_NAME), QOS1);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }
  snprintf(message, sizeof(message),"{\"ts\":%lu,""\"mac\":\"%02X%02X%02X%02X%02X%02X\",""\"status\":\"start\"}",GetRtcEpoch(), MACAddr[0], MACAddr[1], MACAddr[2],MACAddr[3],MACAddr[4],MACAddr[5]);
  len = 0;
  while (message[len] != '\0') {
      len++;
  }
  printf("strlen %i\n\r",len);

  ret = nxd_mqtt_client_publish(&MqttClient, TOPIC_NAME, STRLEN(TOPIC_NAME),(CHAR*)message, len, NX_TRUE, QOS1, NX_WAIT_FOREVER);
      if (ret != NX_SUCCESS)
      {
        Error_Handler();
      }
  tx_event_flags_get(&mqtt_app_flag, DEMO_ALL_EVENTS, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);

  while(1){
	/* Wait for measurement data from other thread */
	ret = tx_queue_receive(&measurement_queue, &sensor_data, TX_WAIT_FOREVER);
	if (ret != TX_SUCCESS) {
	       printf("Failed to receive data from queue: %u\n", ret);
	       continue;
	}
	/* Format JSON message */
	snprintf(message, sizeof(message),"{\"ts\":%lu,\"mac\":\"%02X%02X%02X%02X%02X%02X\",\"nb_detect\":%.0f,\"state_detect\":%i}",sensor_data.timestamp,MACAddr[0], MACAddr[1], MACAddr[2],MACAddr[3], MACAddr[4], MACAddr[5],sensor_data.nb_detect,sensor_data.state_detect);
    /* Publish data */
	len = 0;
	while (message[len] != '\0') {
	    len++;
	}
    /* Publish a message with QoS Level 1. */
    ret = nxd_mqtt_client_publish(&MqttClient, TOPIC_NAME, STRLEN(TOPIC_NAME),(CHAR*)message, len, NX_TRUE, QOS1, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
      Error_Handler();
    }

    /* wait for the broker to publish the message. */
    tx_event_flags_get(&mqtt_app_flag, DEMO_ALL_EVENTS, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);

    /* check event received */
    if(events & DEMO_MESSAGE_EVENT)
    {
      /* get message from the broker */
      ret = nxd_mqtt_client_message_get(&MqttClient, topic_buffer, sizeof(topic_buffer), &topic_length,message_buffer, sizeof(message_buffer), &message_length);
      if(ret == NXD_MQTT_SUCCESS)
      {
        printf("Message %d received: TOPIC = %s, MESSAGE = %s\n", message_count + 1, topic_buffer, message_buffer);
      }
      else
      {
        Error_Handler();
      }
    }

	tx_thread_sleep(100);
    }
}
/**
* @brief  Link thread entry
* @param thread_input: ULONG thread parameter
* @retval none
*/
static VOID App_Link_Thread_Entry(ULONG thread_input)
{
  ULONG actual_status;
  UINT linkdown = 0, status;

  while(1)
  {
    /* Send request to check if the Ethernet cable is connected. */
    status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_LINK_ENABLED,
                                      &actual_status, 10);

    if(status == NX_SUCCESS)
    {
      if(linkdown == 1)
      {
        linkdown = 0;

        /* The network cable is connected. */
        printf("The network cable is connected.\n\r");

        /* Send request to enable PHY Link. */
        nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_ENABLE,
                                      &actual_status);

        /* Send request to check if an address is resolved. */
        status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_ADDRESS_RESOLVED,
                                      &actual_status, 10);
        if(status == NX_SUCCESS)
        {
          /* Stop DHCP */
          nx_dhcp_stop(&DHCPClient);

          /* Reinitialize DHCP */
          nx_dhcp_reinitialize(&DHCPClient);

          /* Start DHCP */
          nx_dhcp_start(&DHCPClient);

          /* wait until an IP address is ready */
          if(tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
          {
            /* USER CODE BEGIN DHCPSemaphore get error */
            Error_Handler();
            /* USER CODE END DHCPSemaphore get error */
          }

          PRINT_IP_ADDRESS(IpAddress);
        }
        else
        {
          /* Set the DHCP Client's remaining lease time to 0 seconds to trigger an immediate renewal request for a DHCP address. */
          nx_dhcp_client_update_time_remaining(&DHCPClient, 0);
        }
      }
    }
    else
    {
      if(0 == linkdown)
      {
        linkdown = 1;
        /* The network cable is not connected. */
        printf("The network cable is not connected.\n\r");
        nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_DISABLE,
                                      &actual_status);
      }
    }

    tx_thread_sleep(NX_APP_CABLE_CONNECTION_CHECK_PERIOD);
  }
}
#include <time.h>

static void SetRtcFromEpoch(uint32_t epoch) {
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef sTime = {0};
    struct tm *tm_ptr = gmtime((time_t*)&epoch);  // Convert epoch to UTC time

    // Time structure
    sTime.Hours = tm_ptr->tm_hour;
    sTime.Minutes = tm_ptr->tm_min;
    sTime.Seconds = tm_ptr->tm_sec;
    sTime.TimeFormat = RTC_HOURFORMAT_24;

    // Date structure
    sDate.WeekDay = (tm_ptr->tm_wday == 0) ? 7 : tm_ptr->tm_wday;  // RTC: Mon=1, Sun=7
    sDate.Month = tm_ptr->tm_mon + 1;  // struct tm: 0-11 → RTC: 1-12
    sDate.Date = tm_ptr->tm_mday;
    sDate.Year = tm_ptr->tm_year - 100;  // struct tm: years since 1900 → RTC: years since 2000

    // Write to RTC
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}
static uint32_t GetRtcEpoch() {
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;
    struct tm tm_time = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    tm_time.tm_year = sDate.Year + 100;  // RTC → struct tm year offset
    tm_time.tm_mon = sDate.Month - 1;
    tm_time.tm_mday = sDate.Date;
    tm_time.tm_hour = sTime.Hours;
    tm_time.tm_min = sTime.Minutes;
    tm_time.tm_sec = sTime.Seconds;

    return (uint32_t)mktime(&tm_time);
}

/**
 * @brief Displays a Welcome screen
 */
static void Display_WelcomeScreen(void)
{
  static uint32_t t0 = 0;
  if (t0 == 0)
    t0 = HAL_GetTick();

  if (HAL_GetTick() - t0 < 4000)
  {
    /* Draw logo */
   // UTIL_LCD_FillRGBRect(300, 100, (uint8_t *) stlogo, 200, 107);

    /* Display welcome message */
    UTIL_LCD_SetBackColor(0x40000000);
    UTIL_LCDEx_PrintfAt(0, LINE(16), 1, "Object detection");
    UTIL_LCDEx_PrintfAt(0, LINE(17), 1, WELCOME_MSG_1);
    UTIL_LCDEx_PrintfAt(0, LINE(18), 1, WELCOME_MSG_2);
    UTIL_LCD_SetBackColor(0);
  }
}

