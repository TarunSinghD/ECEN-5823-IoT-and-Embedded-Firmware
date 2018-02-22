/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Bluetooth mesh light example
 *
 * This example implements a Bluetooth mesh light node.
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* WSTK specific includes */
#include "boards.h"
#include "btMesh-configuration.h"
#include "board_features.h"
#include "retargetserial.h"
#include "graphics.h"

/* Silicon Labs radio board specific includes */
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Included if the Silicon Labs radio board has a SPI flash */
#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

/* emLib (HW drivers) specific includes */
#include <em_gpio.h>

/* Bluetooth LE stack includes */
#include <native_gecko.h>
#include <gecko_configuration.h>

/* Bluetooth LE GATT database */
#include "gatt_db.h"

/* Bluetooth mesh stack includes */
#include "gecko_bgapi_mesh_node_native.h"
#include "mesh_generic_model_capi_types.h"
#include "gecko_bgapi_mesh_generic_server_native.h"
#include "mesh_lib.h"
#include "em_timer.h"
#include "em_cmu.h"

/* EFR32 hardware initialization */
#include "InitDevice.h"

struct mesh_generic_state current, target;

// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

// heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + 8000];

/*
 * Maximum number of Bluetooth advertisement sets.
 * 1 is allocated for Bluetooth LE stack
 * 1 one for Bluetooth mesh stack
 * 1 needs to be allocated for each Bluetooth mesh network
 *   - Currently up to 4 networks are supported at a time
 */
#define MAX_ADVERTISERS (2 + 4)

// Bluetooth stack configuration
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100,
  .gattdb = &bg_gattdb_data,
};

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_PROVISIONING   66
#define TIMER_ID_TRANSITION   55

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */
#define LCD_ROW_NAME     1  /* 1st row, device name */
#define LCD_ROW_STATUS     2    /* 2nd row, node status */
#define LCD_ROW_CONNECTION 3    /* 3rd row, connection status */
#define LCD_ROW_LED_STATUS	4
#define LCD_ROW_COLOR	5
#define LCD_ROW_PASSKEY		6
#define LCD_ROW_MAX        6    /* total number of rows used */

#define LCD_ROW_LEN        32   /* up to 32 characters per each row */

/** global variables */
static uint16 _my_index = 0xffff; /* Index of the Primary Element of the Node */
static uint16 _my_address = 0;    /* Address of the Primary Element of the Node */
static char LCD_data[LCD_ROW_MAX][LCD_ROW_LEN];   /* 2D array for storing the LCD content */
static uint8 num_connections = 0;     /* number of active Bluetooth connections */
static uint8 conn_handle = 0xFF;      /* handle of the last opened LE connection */
uint8 count = 0;

static struct lightbulb_state {
  // On/Off Server state
  uint8_t onoff_current;
  uint8_t onoff_target;

  // Transition Time Server state
  uint8_t transtime;

  // On Power Up Server state
  uint8_t onpowerup;
} lightbulb_state;

/**
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 */
#define LED_STATE_OFF    0   /* light off (both LEDs turned off)   */
#define LED_STATE_ON     1   /* light on (both LEDs turned on)     */
#define LED_STATE_TRANS  2   /* transition (LED0 on, LED1 off)     */
#define LED_STATE_PROV   3   /* provisioning (LEDs blinking)       */

/**
 *  These are needed to support radio boards with active-low and
 *  active-high LED configuration
 */
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif

/* Define PWM frequency value. Value does not represent absolute PWM frequency. PWM_FREQ value 65000
produces ~1kHz PWM signal */
#define PWM_FREQ 65000

/* Define PWM mode. The light is constantly changing or staying constant. Value 1-100 defines constant
brightness and the value define duty-cycle. Values 101-255 sets light to constantly changing brightness
which makes breathing effect. */
uint16_t pwm_mode = 200;

static uint32 pwm_total_period = 0;
static float red_duty_cycle = 1.0;
static float blue_duty_cycle = 1.0;
static float green_duty_cycle = 1.0;
typedef enum
{
	red = 0,
	green = 1,
	blue = 2
} color_e;

static color_e current_color;
static uint8 color = 0;

#define RED_PIN		10
#define BLUE_PIN	11
#define GREEN_PIN	12

/**
 * This is a callback function that is invoked each time a GPIO interrupt in one of the pushbutton
 * inputs occurs. Pin number is passed as parameter.
 *
 * Note: this function is called from ISR context and therefore it is not possible to call any BGAPI
 * functions directly. The button state change is signaled to the application using gecko_external_signal()
 * that will generate an event gecko_evt_system_external_signal_id which is then handled in the main loop.
 */
void gpioint(uint8_t pin)
{
  if (pin == BSP_GPIO_PB0_PIN) {
    gecko_external_signal(0x1);
  } else if (pin == BSP_GPIO_PB1_PIN) {
    gecko_external_signal(0x2);
  }
}

/**
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger an interrupt on the
 * rising edge (button released).
 */
void enable_button_interrupts(void)
{
  GPIOINT_Init();

  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, true, false, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, true, false, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_GPIO_PB0_PIN, gpioint);
  GPIOINT_CallbackRegister(BSP_GPIO_PB1_PIN, gpioint);
}

///**
// * button initialization. Configure pushbuttons PB0,PB1
// * as inputs.
// */
//static void button_init()
//{
//  // configure pushbutton PB0 and PB1 as inputs
//  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 1);
//  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, 1);
//}

/**
 * Update the state of LEDs. Takes one parameter LED_STATE_xxx that defines
 * the new state.
 */
static void LED_set_state(int state)
{
  switch (state) {
    case LED_STATE_OFF:
#if 0
    	color++;
    	            	  current_color = color % 3;
    	            	  if (current_color == red)
    	            	      	  {
    	            	      		  LCD_write("Red", LCD_ROW_COLOR);
    	            	      	  }

    	            	      	  else if (current_color == green)
    	            	      	  {
    	            	      		  LCD_write("Green", LCD_ROW_COLOR);
    	            	      	  }

    	            	      	  else if (current_color == blue)
    	            	      	  {
    	            	      		  LCD_write("Blue", LCD_ROW_COLOR);
    	            	      	  }
#endif
    	LCD_write("LED OFF1", LCD_ROW_LED_STATUS);

      TURN_LED_OFF(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_OFF(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_ON:
#if 0
    	if (current_color == red)
    	            	  {
    	            		  red_duty_cycle -= 0.1;

    	            		  if (red_duty_cycle < 0)
    	            		  {
    	            			  red_duty_cycle = 1.0;
    	            		  }

    	            		  TIMER_CompareBufSet(TIMER0, red, pwm_total_period * red_duty_cycle);
    	            	  }

    	            	  else if (current_color == green)
    	            	  {
    	            		  green_duty_cycle -= 0.1;

    	            		  if (green_duty_cycle < 0)
    	            		  {
    	            			  green_duty_cycle = 1.0;
    	            		  }

    	            		  TIMER_CompareBufSet(TIMER0, green, pwm_total_period * green_duty_cycle);
    	            	  }

    	            	  else if (current_color == blue)
    	            	  {
    	            		  blue_duty_cycle -= 0.1;

    	            		  if (blue_duty_cycle < 0)
    	            		  {
    	            			  blue_duty_cycle = 1.0;
    	            		  }

    	            		  TIMER_CompareBufSet(TIMER0, blue, pwm_total_period * blue_duty_cycle);
    	            	  }
#endif
    	LCD_write("LED ON1", LCD_ROW_LED_STATUS);
      TURN_LED_ON(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_ON(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_TRANS:
      TURN_LED_OFF(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_ON(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_PROV:
      GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;

    default:
      break;
  }
}

#if (EMBER_AF_BOARD_TYPE == BRD4304A)
/**
 * LNA init. Required only for MGM12P radio board (BRD4304A)
 */
#define PRS_CH_CTRL_SIGSEL_RACLNAEN 0x00000003UL
#define PRS_CH_CTRL_SIGSEL_RACPAEN 0x00000004UL
#define PRS_CH_CTRL_SOURCESEL_RAC (0x00000051UL << 8)

static void LNA_init(void)
{
  GPIO_PinModeSet(gpioPortD, 10, gpioModePushPullAlternate, 0);
  GPIO_PinModeSet(gpioPortD, 11, gpioModePushPullAlternate, 0);

  PRS->CH[5].CTRL = PRS_CH_CTRL_SIGSEL_RACLNAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->CH[6].CTRL = PRS_CH_CTRL_ORPREV
                    | PRS_CH_CTRL_SIGSEL_RACPAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH5LOC_LOC0;  // loc0 -> PD10
  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH6LOC_LOC13; // loc12 -> PD11
  PRS->ROUTEPEN |= (PRS_ROUTEPEN_CH5PEN | PRS_ROUTEPEN_CH6PEN);
}
#endif

static int lightbulb_state_load(void);
static int lightbulb_state_store(void);

static uint32_t default_transition_time(void)
{
  return mesh_lib_transition_time_to_ms(lightbulb_state.transtime);
}

static errorcode_t onoff_response(uint16_t element_index,
                                  uint16_t client_addr,
                                  uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = lightbulb_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = lightbulb_state.onoff_target;

  return mesh_lib_generic_server_response(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = lightbulb_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = lightbulb_state.onoff_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static errorcode_t onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
  }

  return e;
}
//typedef enum
//{
//	blue = 0,
//	green = 1,
//	red = 2
//} color_e;
//
//uint8 previous_color = blue;
uint8 oncount = 0;
static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	//duty cycle
	if (request->on_off)
	{
		if (current_color == red)
		    	  {
		    		  red_duty_cycle -= 0.1;

		    		  if (red_duty_cycle < 0)
		    		  {
		    			  red_duty_cycle = 1.0;
		    		  }

		    		  TIMER_CompareBufSet(TIMER0, red, pwm_total_period * red_duty_cycle);


		    	  }

		    	  else if (current_color == green)
		    	  {
		    		  green_duty_cycle -= 0.1;

		    		  if (green_duty_cycle < 0)
		    		  {
		    			  green_duty_cycle = 1.0;
		    		  }

		    		  TIMER_CompareBufSet(TIMER0, green, pwm_total_period * green_duty_cycle);

		    	  }

		    	  else if (current_color == blue)
		    	  {
		    		  blue_duty_cycle -= 0.1;

		    		  if (blue_duty_cycle < 0)
		    		  {
		    			  blue_duty_cycle = 1.0;
		    		  }

		    		  TIMER_CompareBufSet(TIMER0, blue, pwm_total_period * blue_duty_cycle);

		    	  }
//	count++;
//
//	    	    switch (count % 5)
//	    	    {
//	    			case 0:
//	    				LCD_write("OFF0", LCD_ROW_COLOR);
//	    				break;
//
//	    			case 1:
//	    				LCD_write("OFF1", LCD_ROW_COLOR);
//	    				break;
//
//	    			case 2:
//	    				LCD_write("OFF2", LCD_ROW_COLOR);
//	    				break;
//
//	    			case 3:
//	    				LCD_write("OFF3", LCD_ROW_COLOR);
//	    				break;
//
//	    			case 4:
//	    				LCD_write("OFF4", LCD_ROW_COLOR);
//	    				break;
//	    	    }
	}

	//color
	if (!(request->on_off))
		{
		color++;
		    	  current_color = color % 3;

		    	  if (current_color == red)
		    	  {
		    		  LCD_write("Red", LCD_ROW_COLOR);
		    	  }

		    	  else if (current_color == green)
		    	  {
		    		  LCD_write("Green", LCD_ROW_COLOR);
		    	  }

		    	  else if (current_color == blue)
		    	  {
		    		  LCD_write("Blue", LCD_ROW_COLOR);
		    	  }
//		oncount++;
//
//		    	    switch (oncount % 5)
//		    	    {
//		    			case 0:
//		    				LCD_write("ON0", LCD_ROW_COLOR);
//		    				break;
//
//		    			case 1:
//		    				LCD_write("ON1", LCD_ROW_COLOR);
//		    				break;
//
//		    			case 2:
//		    				LCD_write("ON2", LCD_ROW_COLOR);
//		    				break;
//
//		    			case 3:
//		    				LCD_write("ON3", LCD_ROW_COLOR);
//		    				break;
//
//		    			case 4:
//		    				LCD_write("ON4", LCD_ROW_COLOR);
//		    				break;
//		    	    }
		}
  printf("ON/OFF request: requested state=<%s>, transition=%u, delay=%u\r\n",
         request->on_off ? "ON" : "OFF", transition_ms, delay_ms);

  if (lightbulb_state.onoff_current == request->on_off) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Turning lightbulb <%s>\r\n", request->on_off ? "ON" : "OFF");
    if (transition_ms == 0 && delay_ms == 0)
    { // Immediate change
      lightbulb_state.onoff_current = request->on_off;
      lightbulb_state.onoff_target = request->on_off;

      if (request->on_off == 1)
      {
    	  //LCD_write("ON", LCD_ROW_COLOR);
      }

      else if (request->on_off == 0)
      {
    	  //LCD_write("OFF", LCD_ROW_COLOR);
      }

      if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF)
      {
        LED_set_state(LED_STATE_OFF);

      }
      else
      {
        LED_set_state(LED_STATE_ON);
        //LCD_write("ON1", LCD_ROW_COLOR);

      }
    }
    else
    {
      // Current state remains as is for now
      lightbulb_state.onoff_target = request->on_off;
      LED_set_state(LED_STATE_TRANS); // set LEDs to transition mode
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms + transition_ms), TIMER_ID_TRANSITION, 1);
    }
    lightbulb_state_store();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    onoff_response(element_index, client_addr, appkey_index);
  } else {
    onoff_update(element_index);
  }
}

static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
  // TODO
}

static errorcode_t power_onoff_response(uint16_t element_index,
                                        uint16_t client_addr,
                                        uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = lightbulb_state.onpowerup;

  return mesh_lib_generic_server_response(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t power_onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = lightbulb_state.onpowerup;

  return mesh_lib_generic_server_update(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static errorcode_t power_onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = power_onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_power_up);
  }

  return e;
}

static void power_onoff_request(uint16_t model_id,
                                uint16_t element_index,
                                uint16_t client_addr,
                                uint16_t server_addr,
                                uint16_t appkey_index,
                                const struct mesh_generic_request *request,
                                uint32_t transition_ms,
                                uint16_t delay_ms,
                                uint8_t request_flags)
{
  printf("ON POWER UP request received; state=<%s>\n",
         lightbulb_state.onpowerup == 0 ? "OFF"
         : lightbulb_state.onpowerup == 1 ? "ON"
         : "RESTORE");

  if (lightbulb_state.onpowerup == request->on_power_up) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting onpowerup to <%s>\n",
           request->on_power_up == 0 ? "OFF"
           : request->on_power_up == 1 ? "ON"
           : "RESTORE");
    lightbulb_state.onpowerup = request->on_power_up;
    lightbulb_state_store();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    power_onoff_response(element_index, client_addr, appkey_index);
  } else {
    power_onoff_update(element_index);
  }
}

static void power_onoff_change(uint16_t model_id,
                               uint16_t element_index,
                               const struct mesh_generic_state *current,
                               const struct mesh_generic_state *target,
                               uint32_t remaining_ms)
{
  // TODO
}

static errorcode_t transtime_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = lightbulb_state.transtime;

  return mesh_lib_generic_server_response(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t transtime_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = lightbulb_state.transtime;

  return mesh_lib_generic_server_update(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static void transtime_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  printf("TRANSTIME request received; state=<0x%x>\n",
         lightbulb_state.transtime);

  if (lightbulb_state.transtime == request->transition_time) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting transtime to <0x%x>\n", request->transition_time);
    lightbulb_state.transtime = request->transition_time;
    lightbulb_state_store();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    transtime_response(element_index, client_addr, appkey_index);
  } else {
    transtime_update(element_index);
  }
}

static void transtime_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  // TODO
}

/**
 * This function loads the saved light state from Persistent Storage and
 * copies the data in the global variable lightbulb_state
 */
static int lightbulb_state_load(void)
{
  struct gecko_msg_flash_ps_load_rsp_t* pLoad;

  pLoad = gecko_cmd_flash_ps_load(0x4004);

  if (pLoad->result) {
    memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
    return -1;
  }

  memcpy(&lightbulb_state, pLoad->value.data, pLoad->value.len);

  return 0;
}

/**
 * this function saves the current light state in Persistent Storage so that
 * the data is preserved over reboots and power cycles. The light state is hold
 * in a global variable lightbulb_state. a PS key with ID 0x4004 is used to store
 * the whole struct.
 */
static int lightbulb_state_store(void)
{
  struct gecko_msg_flash_ps_save_rsp_t* pSave;

  pSave = gecko_cmd_flash_ps_save(0x4004, sizeof(struct lightbulb_state), (const uint8*)&lightbulb_state);

  if (pSave->result) {
    printf("lightbulb_state_store(): PS save failed, code %x\r\n", pSave->result);
    return(-1);
  }

  return 0;
}

/**
 * Initialization of the models supported by this node. This function registeres callbacks for
 * each of the three supported models.
 */
static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           onoff_request,
                                           onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_POWER_ON_OFF_SETUP_SERVER_MODEL_ID,
                                           0,
                                           power_onoff_request,
                                           power_onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                           0,
                                           transtime_request,
                                           transtime_change);
}

/**
 * Light node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void lightbulb_state_init(void)
{
  /* Initialize mesh lib */
  mesh_lib_init(malloc, free, 8);

  memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
  if (lightbulb_state_load() != 0) {
    printf("lightbulb_state_load() failed, using defaults\r\n");
    goto publish;
  }

  // Handle on power up behavior
  switch (lightbulb_state.onpowerup) {
    case MESH_GENERIC_ON_POWER_UP_STATE_OFF:
      printf("On power up state is OFF\r\n");
      lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
      lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
      LED_set_state(LED_STATE_OFF);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_ON:
      printf("On power up state is ON\r\n");
      lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
      lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_ON;
      LED_set_state(LED_STATE_ON);
      //LCD_write("ON2", LCD_ROW_COLOR);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_RESTORE:
      printf("On power up state is RESTORE\r\n");
      if (lightbulb_state.onoff_current != lightbulb_state.onoff_target) {
        uint32_t transition_ms = default_transition_time();

        printf("Starting on power up transition\r\n");
        LED_set_state(LED_STATE_TRANS);
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_TRANSITION, 1);
      } else {
        printf("Keeping loaded state\r\n");
        if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
          LED_set_state(LED_STATE_OFF);
        } else {
          LED_set_state(LED_STATE_ON);
          //LCD_write("ON3", LCD_ROW_COLOR);
        }
      }
      break;
  }

  lightbulb_state_store();

  publish:
  init_models();
  onoff_update_and_publish(_my_index);
  power_onoff_update_and_publish(_my_index);
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

// this is needed by the LCD driver
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  return 0;
}

/**
 * button initialization. Configure pushbuttons PB0,PB1
 * as inputs.
 */
static void button_init()
{
  // configure pushbutton PB0 and PB1 as inputs
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, 1);
}

/**
 * LED initialization. Configure LED pins as outputs
 */
static void led_init()
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, LED_DEFAULT_STATE);
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, LED_DEFAULT_STATE);
}

/**
 * This function is used to write one line in the LCD. The parameter 'row' selects which line
 * is written, possible values are defined as LCD_ROW_xxx.
 */
void LCD_write(char *str, uint8 row)
{
  char LCD_message[LCD_ROW_MAX * LCD_ROW_LEN];
  char *pRow; /* pointer to selected row */
  int i;

  if (row > LCD_ROW_MAX) {
    return;
  }

  pRow  = &(LCD_data[row - 1][0]);

  sprintf(pRow, str);

  LCD_message[0] = 0;

  for (i = 0; i < LCD_ROW_MAX; i++) {
    pRow  = &(LCD_data[i][0]);
    strcat(LCD_message, pRow);
    strcat(LCD_message, "\n"); // add newline at end of reach row
  }

  graphWriteString(LCD_message);
}

/**
 * LCD initialization, called once at startup.
 */
void LCD_init(void)
{
  /* clear LCD_data table */
  memset(&LCD_data, 0, sizeof(LCD_data));

  /* initialize graphics driver and set the title text */
  graphInit("SILICON LABORATORIES\nBluetooth Mesh Demo\n\n");

  LCD_write("initializing", LCD_ROW_STATUS);
}

/**
 * Set device name in the GATT database. A unique name is genrerated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "light node %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  LCD_write(name, LCD_ROW_NAME);
}



/**
 * This function is called when a light on/off transition has completed
 */
void transition_complete()
{

  // transition done -> set state, update and publish
  lightbulb_state.onoff_current = lightbulb_state.onoff_target;

  printf("transition complete. New state is %s\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");

  if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF)
  {
    LED_set_state(LED_STATE_OFF);


  }
  else
  {
    LED_set_state(LED_STATE_ON);
    //LCD_write("ON4", LCD_ROW_COLOR);
  }

  lightbulb_state_store();
  onoff_update_and_publish(_my_index);
}

/**
 *  this function is called to initiate factory reset. Factory reset may be initiated
 *  by keeping one of the WSTK pushbuttons pressed during reboot. Factory reset is also
 *  performed if it is requested by the provisioner (event gecko_evt_mesh_node_reset_id)
 */
void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  LCD_write("\n***\nFACTORY RESET\n***", LCD_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_endpoint_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 * This function is used to configures PWM modes.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  uint32_t compareValue;

  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);

  compareValue = TIMER_CaptureGet(TIMER0, 0);

  if (pwm_mode > 100)
  {

	  /* increment duty-cycle or reset if reached TOP value */
	  if( compareValue == TIMER_TopGet(TIMER0))
		TIMER_CompareBufSet(TIMER0, 0, 0);
	  else
		TIMER_CompareBufSet(TIMER0, 0, ++compareValue);

	  compareValue = TIMER_CaptureGet(TIMER0, 1);
	  /* decrement duty-cycle or reset if reached MIN value */
	  if( compareValue == 0)
		TIMER_CompareBufSet(TIMER0, 1, TIMER_TopGet(TIMER0));
	  else
		TIMER_CompareBufSet(TIMER0, 1, --compareValue);

	  compareValue = TIMER_CaptureGet(TIMER0, 2);
	  	  /* decrement duty-cycle or reset if reached MIN value */
	  	  if( compareValue == 0)
	  		TIMER_CompareBufSet(TIMER0, 2, TIMER_TopGet(TIMER0));
	  	  else
	  		TIMER_CompareBufSet(TIMER0, 2, --compareValue);
  }
    /* sets the given duty-cycle value to the timer */
  else if ( pwm_mode >= 0 && pwm_mode <= 100)
  {

	  TIMER_CompareBufSet(TIMER0, 1, TIMER_TopGet(TIMER0)*(1-0.01*pwm_mode));
	  TIMER_CompareBufSet(TIMER0, 0, TIMER_TopGet(TIMER0)*(1-0.01*pwm_mode));
	  TIMER_CompareBufSet(TIMER0, 2, TIMER_TopGet(TIMER0)*(1-0.01*pwm_mode));
  }
}

int i = 0;
uint16 passkey = 0;
char password[20];

int main()
{
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  enter_DefaultMode_from_RESET();

#if (EMBER_AF_BOARD_TYPE == BRD4304A)
  LNA_init();
#endif

  gecko_init(&config);

#ifdef FEATURE_PTI_SUPPORT
  APP_ConfigEnablePti();
#endif // FEATURE_PTI_SUPPORT

  RETARGET_SerialInit();

  /* initialize LEDs and buttons. Note: some radio boards share the same GPIO for button & LED.
   * Initialization is done in this order so that default configuration will be "button" for those
   * radio boards with shared pins. led_init() is called later as needed to (re)initialize the LEDs
   * */
  led_init();
  button_init();

  LCD_init();

  /* Initialize peripherals */
    //enter_DefaultMode_from_RESET();

    /* Initialize stack */
    //gecko_init(&config);

    /* Enable clock for GPIO module */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Enable clock for TIMER0 module */
    CMU_ClockEnable(cmuClock_TIMER0, true);

    /* Initialize pins used for PWM */
    /* Pin PF6 is configured to Push-pull */
    GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BSP_GPIO_LED0_PORT, 4, gpioModePushPull, 0);

    /* Pin PF7 is configured to Push-pull */
    GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(BSP_GPIO_LED1_PORT, 5, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortD, RED_PIN, gpioModePushPull, 0); //CC0
         GPIO_PinModeSet(gpioPortD, GREEN_PIN, gpioModePushPull, 0); //CC1
         GPIO_PinModeSet(gpioPortD, BLUE_PIN, gpioModePushPull, 0); //CC2

         /* Route pins to timer */
         // $[TIMER0 I/O setup]
         /* Set up CC0 */
         TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC0LOC_MASK))
            					| TIMER_ROUTELOC0_CC0LOC_LOC18;
         TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC0PEN;
         /* Set up CC1 */
         TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC1LOC_MASK))
            					| TIMER_ROUTELOC0_CC1LOC_LOC18;
         TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC1PEN;
         // [TIMER0 I/O setup]$

         TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC2LOC_MASK))
            							| TIMER_ROUTELOC0_CC2LOC_LOC18;
         TIMER0->ROUTEPEN = TIMER0->ROUTEPEN | TIMER_ROUTEPEN_CC2PEN;



    /* Select CC channel parameters */
    TIMER_InitCC_TypeDef timerCCInit =
    {
      .eventCtrl  = timerEventEveryEdge,
      .edge       = timerEdgeBoth,
      .prsSel     = timerPRSSELCh0,
      .cufoa      = timerOutputActionNone,
      .cofoa      = timerOutputActionNone,
      .cmoa       = timerOutputActionToggle,
      .mode       = timerCCModePWM,
      .filter     = false,
      .prsInput   = false,
      .coist      = false,
      .outInvert  = false,
    };

    /* Configure CC channel 0 */
    TIMER_InitCC(TIMER0, 0, &timerCCInit);

    /* Configure CC channel 1 */
    TIMER_InitCC(TIMER0, 1, &timerCCInit);

    TIMER_InitCC(TIMER0, 2, &timerCCInit);

    /* Set Top Value */
    TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ);

    /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
    TIMER_CompareBufSet(TIMER0, 0, TIMER_TopGet(TIMER0));

    pwm_total_period = TIMER_TopGet(TIMER0);

    /* Set compare value starting at top value - it will be decremented in the interrupt handler */
    TIMER_CompareBufSet(TIMER0, 1, TIMER_TopGet(TIMER0));
    TIMER_CompareBufSet(TIMER0, 2, TIMER_TopGet(TIMER0));

    /* Select timer parameters */
    TIMER_Init_TypeDef timerInit =
    {
      .enable     = true,
      .debugRun   = true,
      .prescale   = timerPrescale64,
      .clkSel     = timerClkSelHFPerClk,
      .fallAction = timerInputActionNone,
      .riseAction = timerInputActionNone,
      .mode       = timerModeUp,
      .dmaClrAct  = false,
      .quadModeX4 = false,
      .oneShot    = false,
      .sync       = false,
    };

//    /* Enable overflow interrupt */
//    TIMER_IntEnable(TIMER0, TIMER_IF_OF);
//
//    /* Enable TIMER0 interrupt vector in NVIC */
//    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Configure timer */
    TIMER_Init(TIMER0, &timerInit);

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
  }
}

/**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events are handled here.
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  struct gecko_bgapi_mesh_node_cmd_packet *node_evt;
  struct gecko_bgapi_mesh_generic_server_cmd_packet *server_evt;
  struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;

  if (NULL == evt) {
    return;
  }

  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	// initiate_factory_reset();
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
    	//initiate_factory_reset();
      if (GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == 0 || GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN) == 0) {
        initiate_factory_reset();
      } else {
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

        set_device_name(&pAddr->address);


        // Initialize Mesh stack in Node operation mode, wait for initialized event
        //gecko_cmd_mesh_node_init();
        gecko_cmd_mesh_node_init_oob(0x01, 0x02, 0x03, 0x04, 0x02, 0x04, 0x01);

        // re-initialize LEDs (needed for those radio board that share same GPIO for button/LED)
        led_init();
      }
      break;

    case gecko_evt_mesh_node_display_output_oob_id:
            {
                struct gecko_msg_mesh_node_display_output_oob_evt_t *pOOB = (struct gecko_msg_mesh_node_display_output_oob_evt_t *)&(evt->data);
                printf("gecko_msg_mesh_node_display_output_oob_evt_t: action %d, size %d\r\n", pOOB->output_action, pOOB->output_size);

                for(i=0;i<pOOB->data.len;i++)
                {
                    printf("%2.2x ", pOOB->data.data[i]);
                }
                printf("\r\n");

                passkey = (pOOB->data.data[14] << 8) | (pOOB->data.data[15]);
                sprintf(password, "%d", passkey);

                LCD_write(password, LCD_ROW_PASSKEY);
            }
                break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          LED_set_state(LED_STATE_PROV);
          break;

        case TIMER_ID_TRANSITION:
          /* light state transition has completed */
          transition_complete();
          break;

        default:
          break;
      }

      break;

    case gecko_evt_mesh_node_initialized_id:
    	gecko_cmd_system_set_tx_power(-260);

      printf("node initialized\r\n");

      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

      if (pData->provisioned) {
        printf("node is provisioned. address:%x, ivi:%d\r\n", pData->address, pData->ivi);

        _my_address = pData->address;
        _my_index = 0;   // index of primary element hardcoded to zero in this example
        lightbulb_state_init();

        printf("Light initial state is <%s>\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");
        LCD_write("provisioned", LCD_ROW_STATUS);
      } else {
        printf("node is unprovisioned\r\n");
        LCD_write("unprovisioned", LCD_ROW_STATUS);

        printf("starting unprovisioned beaconing...\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      }
      break;

    case gecko_evt_mesh_node_provisioning_started_id:
      printf("Started provisioning\r\n");
      LCD_write("provisioning...", LCD_ROW_STATUS);
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      _my_index = 0;   // index of primary element hardcoded to zero in this example
      lightbulb_state_init();
      printf("node provisioned, got index=%x\r\n", _my_index);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
      LED_set_state(LED_STATE_OFF);
      LCD_write("provisioned", LCD_ROW_STATUS);
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
      printf("provisioning failed, code %x\r\n", prov_fail_evt->result);
      LCD_write("prov failed", LCD_ROW_STATUS);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_node_key_added_id:
      node_evt = (struct gecko_bgapi_mesh_node_cmd_packet *)evt;
      printf("got new %s key with index %x\r\n", node_evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             node_evt->data.evt_mesh_node_key_added.index);
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      break;

    case gecko_evt_mesh_generic_server_client_request_id:

      printf("evt gecko_evt_mesh_generic_server_client_request_id\r\n");
      server_evt = (struct gecko_bgapi_mesh_generic_server_cmd_packet *)evt;
      mesh_lib_generic_server_event_handler(server_evt);
      break;

    case gecko_evt_mesh_node_reset_id:
      printf("evt gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    // just for debugging...
    case gecko_evt_le_gap_adv_timeout_id:
    case gecko_evt_le_gap_bt5_adv_timeout_id:
      // adv timeout events silently discarded
      break;

    case gecko_evt_le_connection_bt5_opened_id:
      printf("evt:gecko_evt_le_connection_bt5_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_bt5_opened.connection;
      // Connection interval of 60(0x3C) * 1.5 = 75 ms
                    	// Slave Latency of 75 * (4 + 1) = 375 ms
                    	// Supervision time out of 600(0x258) ms * 10 = 6 sec
                    	//connect_handle = evt->data.evt_le_connection_opened.connection;
                    	gecko_cmd_le_connection_set_parameters(conn_handle, 0x003C, 0x003C, 0x0004, 0x0258);
      LCD_write("connected", LCD_ROW_CONNECTION);
      break;

    case gecko_evt_le_connection_parameters_id:
      printf("evt:gecko_evt_le_connection_parameters_id\r\n");
      break;

    case gecko_evt_le_connection_closed_id:
      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          LCD_write("", LCD_ROW_CONNECTION);
        }
      }
      break;

    default:
      printf("unhandled evt: %x\r\n", evt_id);
      break;
  }
}
