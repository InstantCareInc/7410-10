/** @file
 * @brief BLE manager implementation
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(ble_manager, LOG_LEVEL_INF);

#include "ble_manager.h"

/* ---------------------------------------------------------
 *  Device Name Definition
 * --------------------------------------------------------- */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ---------------------------------------------------------
 *  Characteristic Storage
 * --------------------------------------------------------- */
static uint8_t accel_cfg = 0x01;
static int16_t accel_xyz[3] = {0};

static uint8_t pressure_cfg = 0x01;
static int16_t pressure_xyz[3] = {0};

static uint8_t button_event = 0;
static uint8_t led1_state = 0;
static uint8_t led2_state = 0;

/* ---------------------------------------------------------
 *  Notification Permit Flags
 * --------------------------------------------------------- */
static bool notify_accel_enabled = false;
static bool notify_pressure_enabled = false;
static bool notify_button_enabled = false;

/* ---------------------------------------------------------
 *  Advertising
 * --------------------------------------------------------- */
const struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN |
                        BT_LE_ADV_OPT_USE_IDENTITY,
                    BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2,
                    NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ACCEL_SERVICE_VAL),
};

static struct k_work adv_work;

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }
    else
    {
        LOG_INF("Advertising started");
    }
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/* ---------------------------------------------------------
 *  Connection Callbacks
 * --------------------------------------------------------- */
static void recycled_cb(void)
{
    printk("Connection object available from previous conn. Disconnect is complete!\n");
    k_work_submit(&adv_work);
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .recycled = recycled_cb,
};

/* ---------------------------------------------------------
 *  CCCD Callbacks
 * --------------------------------------------------------- */
static void accel_ccc_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_accel_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Accel notify: %d", notify_accel_enabled);
}

static void pressure_ccc_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_pressure_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Pressure notify: %d", notify_pressure_enabled);
}

static void button_ccc_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_button_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Button notify: %d", notify_button_enabled);
}

/* ---------------------------------------------------------
 *  LED Write Handler
 * --------------------------------------------------------- */
static ssize_t write_led1(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags)
{
    if (len != 1)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    led1_state = ((uint8_t *)buf)[0];
    LOG_INF("LED1 = %d", led1_state);
    return len;
}

static ssize_t write_led2(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags)
{
    if (len != 1)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    led2_state = ((uint8_t *)buf)[0];
    LOG_INF("LED2 = %d", led2_state);
    return len;
}

/* ---------------------------------------------------------
 *  GATT Services
 * --------------------------------------------------------- */
BT_GATT_SERVICE_DEFINE(accel_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_ACCEL_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_DATA, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ, NULL, NULL, accel_xyz),
                       BT_GATT_CCC(accel_ccc_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_CFG, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, &accel_cfg), );

BT_GATT_SERVICE_DEFINE(press_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_PRESS_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_PRESS_DATA, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ, NULL, NULL, pressure_xyz),
                       BT_GATT_CCC(pressure_ccc_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_PRESS_CFG, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, &pressure_cfg), );

BT_GATT_SERVICE_DEFINE(button_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_BUTTON_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_BUTTON_EVENT, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, &button_event),
                       BT_GATT_CCC(button_ccc_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

BT_GATT_SERVICE_DEFINE(led_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_LED_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_LED1_STATE, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, write_led1, &led1_state),
                       BT_GATT_CHARACTERISTIC(BT_UUID_LED2_STATE, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, write_led2, &led2_state), );

/* ---------------------------------------------------------
 *  Public API to Send Notifications
 * --------------------------------------------------------- */
void ble_notify_accel(void)
{
    if (notify_accel_enabled)
    {
        bt_gatt_notify(NULL, &accel_svc.attrs[1], accel_xyz, sizeof(accel_xyz));
    }
}

void ble_notify_pressure(void)
{
    if (notify_pressure_enabled)
    {
        bt_gatt_notify(NULL, &press_svc.attrs[1], pressure_xyz, sizeof(pressure_xyz));
    }
}

void ble_notify_button(uint8_t event)
{
    button_event = event;

    if (notify_button_enabled)
    {
        bt_gatt_notify(NULL, &button_svc.attrs[1], &button_event, sizeof(button_event));
    }
}

/* ---------------------------------------------------------
 *  BLE Initialization
 * --------------------------------------------------------- */
int ble_manager_init(void)
{
    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    bt_conn_cb_register(&connection_callbacks);

    LOG_INF("Bluetooth initialized");

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    return 0;
}
