#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <ram_pwrdn.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include "zb_mem_config_custom.h"
#include "zb_dimmer_switch.h"

/* Source endpoint used to control light bulb. */
#define LIGHT_SWITCH_ENDPOINT_1 1
#define LIGHT_SWITCH_ENDPOINT_2 2
#define LIGHT_SWITCH_ENDPOINT_3 3
#define LIGHT_SWITCH_ENDPOINT_4 4
#define LIGHT_SWITCH_ENDPOINT_5 5

/* Delay between the light switch startup and light bulb finding procedure. */
#define MATCH_DESC_REQ_START_DELAY K_SECONDS(2)
/* Timeout for finding procedure. */
#define MATCH_DESC_REQ_TIMEOUT K_SECONDS(5)
/* Find only non-sleepy device. */
#define MATCH_DESC_REQ_ROLE ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE

/* Do not erase NVRAM to save the network parameters after device reboot or
 * power-off. NOTE: If this option is set to ZB_TRUE then do full device erase
 * for all network devices before running other samples.
 */
#define ERASE_PERSISTENT_CONFIG ZB_FALSE

/* Dim step size - increases/decreses current level (range 0x000 - 0xfe). */
#define DIMM_STEP 15

/* Transition time for a single step operation in 0.1 sec units.
 * 0xFFFF - immediate change.
 */
#define DIMM_TRANSACTION_TIME 2

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

struct bulb_context
{
	zb_uint8_t endpoint;
	zb_uint16_t short_addr;
	struct k_timer find_alarm;
};

struct zb_device_ctx
{
	zb_zcl_basic_attrs_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
};

static struct bulb_context bulb_ctx;
static struct zb_device_ctx dev_ctx;

/* Declare attribute list for Basic cluster (server). */
ZB_ZCL_DECLARE_BASIC_SERVER_ATTRIB_LIST(
	basic_server_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.power_source);

/* Declare attribute list for Identify cluster (client). */
ZB_ZCL_DECLARE_IDENTIFY_CLIENT_ATTRIB_LIST(
	identify_client_attr_list);

/* Declare attribute list for Identify cluster (server). */
ZB_ZCL_DECLARE_IDENTIFY_SERVER_ATTRIB_LIST(
	identify_server_attr_list,
	&dev_ctx.identify_attr.identify_time);

/* Declare attribute list for On/Off cluster (client). */
ZB_ZCL_DECLARE_ON_OFF_CLIENT_ATTRIB_LIST(
	on_off_client_attr_list);

/* Declare attribute list for Level control cluster (client). */
ZB_ZCL_DECLARE_LEVEL_CONTROL_CLIENT_ATTRIB_LIST(
	level_control_client_attr_list);

/* Declare cluster list for Dimmer Switch device. */
ZB_DECLARE_DIMMER_SWITCH_CLUSTER_LIST(
	dimmer_switch_clusters,
	basic_server_attr_list,
	identify_client_attr_list,
	identify_server_attr_list,
	NULL,
	NULL,
	on_off_client_attr_list,
	level_control_client_attr_list);

/* Declare endpoint for Dimmer Switch device. */
ZB_DECLARE_DIMMER_SWITCH_EP(
	dimmer_switch_ep_1,
	LIGHT_SWITCH_ENDPOINT_1,
	dimmer_switch_clusters);

ZB_DECLARE_DIMMER_SWITCH_EP(
	dimmer_switch_ep_2,
	LIGHT_SWITCH_ENDPOINT_2,
	dimmer_switch_clusters);

ZB_DECLARE_DIMMER_SWITCH_EP(
	dimmer_switch_ep_3,
	LIGHT_SWITCH_ENDPOINT_3,
	dimmer_switch_clusters);

// ZB_DECLARE_DIMMER_SWITCH_EP(
// 	dimmer_switch_ep_4,
// 	LIGHT_SWITCH_ENDPOINT_4,
// 	dimmer_switch_clusters);

// ZB_DECLARE_DIMMER_SWITCH_EP(
// 	dimmer_switch_ep_5,
// 	LIGHT_SWITCH_ENDPOINT_5,
// 	dimmer_switch_clusters);

// Declare application's device context (list of registered endpoints) for Dimmer Switch device.
// ZBOSS_DECLARE_DEVICE_CTX_1_EP(dimmer_switch_ctx, dimmer_switch_ep_1);
// ZBOSS_DECLARE_DEVICE_CTX_2_EP(dimmer_switch_ctx, dimmer_switch_ep_1, dimmer_switch_ep_2);
ZBOSS_DECLARE_DEVICE_CTX_3_EP(dimmer_switch_ctx, dimmer_switch_ep_1, dimmer_switch_ep_2, dimmer_switch_ep_3);
// ZBOSS_DECLARE_DEVICE_CTX_EP_VA(dimmer_switch_ctx, dimmer_switch_ep_1, dimmer_switch_ep_2, dimmer_switch_ep_3, dimmer_switch_ep_4, dimmer_switch_ep_5);

int light_switch_endpoint;

// =================================================== uart ===============================================================
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#define SLEEP_TIME_MS 1000
#define RECEIVE_BUFF_SIZE 9
#define RECEIVE_TIMEOUT 100

/* STEP 4.1 - Get the device pointer of the UART hardware */
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* STEP 9.1 - Define the transmission buffer, which is a buffer to hold the data to be sent over UART */
static uint8_t tx_buf[] = {"nRF Connect SDK Fundamentals Course\n\r"
						   "Press 1/0 on your keyboard to toggle bulb on zigbee network\n\r"};

/* STEP 10.1.2 - Define the receive buffer */
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

// ==================================================================================================================

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sig_hndler = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &sig_hndler);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	switch (sig)
	{
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
	/* fall-through */
	case ZB_BDB_SIGNAL_STEERING:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		if (status == RET_OK)
			/* Check the light device address. */
			if (bulb_ctx.short_addr == 0xFFFF)
				k_timer_start(&bulb_ctx.find_alarm, MATCH_DESC_REQ_START_DELAY, MATCH_DESC_REQ_TIMEOUT);
		break;
	case ZB_ZDO_SIGNAL_LEAVE:
		/* If device leaves the network, reset bulb short_addr. */
		if (status == RET_OK)
		{
			zb_zdo_signal_leave_params_t *leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(sig_hndler, zb_zdo_signal_leave_params_t);

			if (leave_params->leave_type == ZB_NWK_LEAVE_TYPE_RESET)
				bulb_ctx.short_addr = 0xFFFF;
		}

		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;

	default:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	}

	if (bufid)
		zb_buf_free(bufid);
}

// ==================================================================================================================

/**@brief Starts identifying the device.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void start_identifying(zb_bufid_t bufid)
{
	LOG_INF("start identifying function called");
	ZVUNUSED(bufid);

	if (ZB_JOINED())
	{
		/* Check if endpoint is in identifying mode,
		 * if not, put desired endpoint in identifying mode.
		 */
		if (dev_ctx.identify_attr.identify_time == ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE)
		{
			LOG_INF("Endpoint: %d", light_switch_endpoint);
			zb_ret_t zb_err_code = zb_bdb_finding_binding_target(light_switch_endpoint);

			if (zb_err_code == RET_OK)
				LOG_INF("Enter identify mode");
			else if (zb_err_code == RET_INVALID_STATE)
				LOG_INF("RET_INVALID_STATE - Cannot enter identify mode");
			else
			{
				LOG_INF("error in start identifying");
				ZB_ERROR_CHECK(zb_err_code);
			}
		}
		else
		{
			LOG_INF("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	}
	else
		LOG_INF("Device not in a network - cannot enter identify mode");
}

/*@brief Function for sending step requests to the light bulb.
 *
 * @param[in]   bufid        Non-zero reference to Zigbee stack buffer that
 *                           will be used to construct step request.
 * @param[in]   cmd_id       ZCL command id.
 */
static void light_switch_send_step(zb_bufid_t bufid, zb_uint16_t cmd_id)
{
	LOG_INF("Send step level command: %d", cmd_id);
	LOG_INF("Endpoint: %d", light_switch_endpoint);

	ZB_ZCL_LEVEL_CONTROL_SEND_STEP_REQ(bufid,
									   bulb_ctx.short_addr,
									   ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
									   bulb_ctx.endpoint,
									   light_switch_endpoint,
									   ZB_AF_HA_PROFILE_ID,
									   ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
									   NULL,
									   cmd_id,
									   DIMM_STEP,
									   DIMM_TRANSACTION_TIME);
}

// @brief Function for sending ON/OFF requests to the light bulb.
static void light_switch_send_on_off(zb_bufid_t bufid, zb_uint16_t cmd_id)
{
	LOG_INF("Send ON/OFF command in switch_func: %d", cmd_id);
	LOG_INF("Endpoint: %d", light_switch_endpoint);

	ZB_ZCL_ON_OFF_SEND_REQ(bufid,
						   bulb_ctx.short_addr,
						   ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
						   bulb_ctx.endpoint,
						   light_switch_endpoint,
						   ZB_AF_HA_PROFILE_ID,
						   ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
						   cmd_id,
						   NULL);
}

// ==================================================================================================================

void toggle_light(struct uart_event *evt)
{
	zb_uint16_t cmd_id;
	zb_ret_t zb_err_code;

	if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x00)
		cmd_id = ZB_ZCL_CMD_ON_OFF_OFF_ID;
	else if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x01)
		cmd_id = ZB_ZCL_CMD_ON_OFF_ON_ID;

	LOG_INF("String: %s", evt->data.rx.buf);
	zb_err_code = zb_buf_get_out_delayed_ext(light_switch_send_on_off, cmd_id, 0);
	ZB_ERROR_CHECK(zb_err_code);
}

void dim_controller(struct uart_event *evt)
{
	zb_uint16_t cmd_id;
	zb_ret_t zb_err_code;

	if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x00)
		cmd_id = ZB_ZCL_LEVEL_CONTROL_STEP_MODE_DOWN;
	else if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x01)
		cmd_id = ZB_ZCL_LEVEL_CONTROL_STEP_MODE_UP;

	/* Allocate output buffer and send step command. */
	zb_err_code = zb_buf_get_out_delayed_ext(light_switch_send_step, cmd_id, 0);
	if (!zb_err_code)
		LOG_WRN("Buffer is full");
}

void factoryReset_pairing_mode(struct uart_event *evt)
{
	if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x00)
		ZB_SCHEDULE_APP_CALLBACK(zb_bdb_reset_via_local_action, 0);
	else if (evt->data.rx.buf[evt->data.rx.offset + 8] == 0x01)
		ZB_SCHEDULE_APP_CALLBACK(start_identifying, 0);
}

// ==================================================================================================================

/* STEP 7 - Define the callback function for UART */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

	/* Inform default signal handler about user input at the device. */
	user_input_indicate();

	if (bulb_ctx.short_addr == 0xFFFF)
	{
		LOG_INF("No bulb found yet.");
		return;
	}

	switch (evt->type)
	{
	case UART_RX_RDY:
		/* For ON/OFF of Light */
		if (evt->data.rx.buf[evt->data.rx.offset] == 0x5A)
		{
			// ============================================ for LIGHT_SWITCH_ENDPOINT_1 ============================================
			if (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x15)
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_1;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				toggle_light(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x16) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x17))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_1;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				dim_controller(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x18) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x19))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_1;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				factoryReset_pairing_mode(evt);
			}

			// ============================================ for LIGHT_SWITCH_ENDPOINT_2 ============================================
			else if (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x25)
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_2;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				toggle_light(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x26) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x27))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_2;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				dim_controller(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x28) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x29))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_2;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				factoryReset_pairing_mode(evt);
			}

			// ============================================ for LIGHT_SWITCH_ENDPOINT_3 ============================================
			else if (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x35)
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_3;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				toggle_light(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x36) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x37))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_3;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				dim_controller(evt);
			}
			else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x38) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x39))
			{
				light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_3;
				LOG_INF("Endpoint: %d", light_switch_endpoint);
				factoryReset_pairing_mode(evt);
			}

			// // ============================================ for LIGHT_SWITCH_ENDPOINT_4 ============================================
			// else if (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x45)
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_4;
			// 	toggle_light(evt);
			// }
			// else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x46) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x47))
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_4;
			// 	dim_controller(evt);
			// }
			// else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x48) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x49))
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_4;
			// 	factoryReset_pairing_mode(evt);
			// }

			// // ============================================ for LIGHT_SWITCH_ENDPOINT_5 ============================================
			// else if (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x55)
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_5;
			// 	toggle_light(evt);
			// }
			// else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x56) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x57))
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_5;
			// 	dim_controller(evt);
			// }
			// else if ((evt->data.rx.buf[evt->data.rx.offset + 4] == 0x58) || (evt->data.rx.buf[evt->data.rx.offset + 4] == 0x59))
			// {
			// 	light_switch_endpoint = LIGHT_SWITCH_ENDPOINT_5;
			// 	factoryReset_pairing_mode(evt);
			// }

			else
				LOG_INF("VP error");
		}
		else
			LOG_INF("First hex is not 0x5A");

		break;

	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;
	default:
		break;
	}
}

// ==================================================================================================================

// @brief Callback function receiving finding procedure results.
static void find_light_bulb_cb(zb_bufid_t bufid)
{
	LOG_INF("find_light_bulb_cb called");

	/* Get the beginning of the response. */
	zb_zdo_match_desc_resp_t *resp =
		(zb_zdo_match_desc_resp_t *)zb_buf_begin(bufid);
	/* Get the pointer to the parameters buffer, which stores APS layer
	 * response.
	 */
	zb_apsde_data_indication_t *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);
	zb_uint8_t *match_ep;

	if ((resp->status == ZB_ZDP_STATUS_SUCCESS) && (resp->match_len > 0) && (bulb_ctx.short_addr == 0xFFFF))
	{
		/* Match EP list follows right after response header. */
		match_ep = (zb_uint8_t *)(resp + 1);

		/* We are searching for exact cluster, so only 1 EP
		 * may be found.
		 */
		bulb_ctx.endpoint = *match_ep;
		bulb_ctx.short_addr = ind->src_addr;

		LOG_INF("Found bulb addr: %d ep: %d",
				bulb_ctx.short_addr,
				bulb_ctx.endpoint);

		k_timer_stop(&bulb_ctx.find_alarm);
	}

	else
		LOG_INF("Bulb not found, try again");

	if (bufid)
		zb_buf_free(bufid);
}

/**@brief Function for sending ON/OFF and Level Control find request.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer that will be used to
 *                      construct find request.
 */
static void find_light_bulb(zb_bufid_t bufid)
{
	LOG_INF("find_light_bulb called");

	zb_zdo_match_desc_param_t *req;
	zb_uint8_t tsn = ZB_ZDO_INVALID_TSN;

	/* Initialize pointers inside buffer and reserve space for
	 * zb_zdo_match_desc_param_t request.
	 */
	req = zb_buf_initial_alloc(bufid, sizeof(zb_zdo_match_desc_param_t) + (1) * sizeof(zb_uint16_t));

	req->nwk_addr = MATCH_DESC_REQ_ROLE;
	req->addr_of_interest = MATCH_DESC_REQ_ROLE;
	req->profile_id = ZB_AF_HA_PROFILE_ID;

	/* We are searching for 2 clusters: On/Off and Level Control Server. */
	req->num_in_clusters = 2;
	req->num_out_clusters = 0;
	req->cluster_list[0] = ZB_ZCL_CLUSTER_ID_ON_OFF;
	req->cluster_list[1] = ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;

	/* Set 0xFFFF to reset short address in order to parse
	 * only one response.
	 */
	bulb_ctx.short_addr = 0xFFFF;
	tsn = zb_zdo_match_desc_req(bufid, find_light_bulb_cb);

	/* Free buffer if failed to send a request. */
	if (tsn == ZB_ZDO_INVALID_TSN)
	{
		zb_buf_free(bufid);
		LOG_INF("Failed to send Match Descriptor request");
	}
}

/**@brief Find bulb alarm handler.
 *
 * @param[in]   timer   Address of timer.
 */
static void find_light_bulb_alarm(struct k_timer *timer)
{
	LOG_INF("find_light_bulb_alarm called");
	ZB_ERROR_CHECK(zb_buf_get_out_delayed(find_light_bulb));
}

// ==================================================================================================================

int main(void)
{
	int ret;

	/* STEP 4.2 - Verify that the UART device is ready */
	if (!device_is_ready(uart))
	{
		printk("UART device not ready\r\n");
		return 1;
	}

	/* STEP 8 - Register the UART callback function */
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret)
		return 1;

	/* STEP 9.2 - Send the data over UART by calling uart_tx() */
	ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
	if (ret)
		return 1;

	/* STEP 10.3  - Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer */
	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (ret)
		return 1;

	// ==================================================================================================================

	LOG_INF("Starting ZBOSS Light Switch example");

	k_timer_init(&bulb_ctx.find_alarm, find_light_bulb_alarm, NULL);

	zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
	zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

	/* Set default bulb short_addr. */
	bulb_ctx.short_addr = 0xFFFF;

	/* Power off unused sections of RAM to lower device power consumption. */
	if (IS_ENABLED(CONFIG_RAM_POWER_DOWN_LIBRARY))
		power_down_unused_ram();

	/* Register dimmer switch device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&dimmer_switch_ctx);

	/* Basic cluster attributes data. */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_UNKNOWN;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* Start Zigbee default thread. */
	zigbee_enable();

	// ==================================================================================================================

	LOG_INF("ZBOSS Light Switch example started");
	while (1)
	{
		k_sleep(K_FOREVER);
		// k_msleep(SLEEP_TIME_MS);
	}
}