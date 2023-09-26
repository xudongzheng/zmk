/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zmk/usb.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/event_manager.h>
#include <zmk/workqueue.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device *hid_dev;

struct usb_hid_msg {
    uint8_t data[CONFIG_HID_INTERRUPT_EP_MPS];
    uint8_t len;
};

// Keep track of the number of consecutive HID writes failures so messages can
// be dropped if they continuously fail to send.
static int usb_hid_failed;

K_MSGQ_DEFINE(usb_hid_msgq, sizeof(struct usb_hid_msg), 8, 1);

static void usb_hid_work_handler(struct k_work *work) {
    struct usb_hid_msg msg;
    while (k_msgq_peek(&usb_hid_msgq, &msg) == 0) {
        // Attempt to write HID message. If it fails, retry with up to a total
        // of three attempts. Reattempt after 10ms or when the USB HID interrupt
        // IN endpoint is ready, whichever comes first.
        int err = hid_int_ep_write(hid_dev, msg.data, msg.len, NULL);
        if (err) {
            usb_hid_failed++;
            if (usb_hid_failed < 3) {
                k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(),
                                            k_work_delayable_from_work(work), K_MSEC(10));
                return;
            } else {
                LOG_ERR("dropped HID message due to %d consecutive failures", usb_hid_failed);
            }
        }

        // Remove message from message queue and reset failure count.
        k_msgq_get(&usb_hid_msgq, &msg, K_NO_WAIT);
        usb_hid_failed = 0;
    }
}

K_WORK_DELAYABLE_DEFINE(usb_hid_work, usb_hid_work_handler);

static void in_ready_cb(const struct device *dev) {
    k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), &usb_hid_work, K_NO_WAIT);
}

#define HID_GET_REPORT_TYPE_MASK 0xff00
#define HID_GET_REPORT_ID_MASK 0x00ff

#define HID_REPORT_TYPE_INPUT 0x100
#define HID_REPORT_TYPE_OUTPUT 0x200
#define HID_REPORT_TYPE_FEATURE 0x300

#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
static uint8_t hid_protocol = HID_PROTOCOL_REPORT;

static void set_proto_cb(const struct device *dev, uint8_t protocol) { hid_protocol = protocol; }

void zmk_usb_hid_set_protocol(uint8_t protocol) { hid_protocol = protocol; }
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

static uint8_t *get_keyboard_report(size_t *len) {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    if (hid_protocol != HID_PROTOCOL_REPORT) {
        zmk_hid_boot_report_t *boot_report = zmk_hid_get_boot_report();
        *len = sizeof(*boot_report);
        return (uint8_t *)boot_report;
    }
#endif
    struct zmk_hid_keyboard_report *report = zmk_hid_get_keyboard_report();
    *len = sizeof(*report);
    return (uint8_t *)report;
}

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {

    /*
     * 7.2.1 of the HID v1.11 spec is unclear about handling requests for reports that do not exist
     * For requested reports that aren't input reports, return -ENOTSUP like the Zephyr subsys does
     */
    if ((setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_INPUT) {
        LOG_ERR("Unsupported report type %d requested", (setup->wValue & HID_GET_REPORT_TYPE_MASK)
                                                            << 8);
        return -ENOTSUP;
    }

    switch (setup->wValue & HID_GET_REPORT_ID_MASK) {
    case ZMK_HID_REPORT_ID_KEYBOARD: {
        *data = get_keyboard_report(len);
        break;
    }
    case ZMK_HID_REPORT_ID_CONSUMER: {
        struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();
        *data = (uint8_t *)report;
        *len = sizeof(*report);
        break;
    }
    default:
        LOG_ERR("Invalid report ID %d requested", setup->wValue & HID_GET_REPORT_ID_MASK);
        return -EINVAL;
    }

    return 0;
}

static int set_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    if ((setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_OUTPUT) {
        LOG_ERR("Unsupported report type %d requested",
                (setup->wValue & HID_GET_REPORT_TYPE_MASK) >> 8);
        return -ENOTSUP;
    }

    switch (setup->wValue & HID_GET_REPORT_ID_MASK) {
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    case ZMK_HID_REPORT_ID_LEDS:
        if (*len != sizeof(struct zmk_hid_led_report)) {
            LOG_ERR("LED set report is malformed: length=%d", *len);
            return -EINVAL;
        } else {
            struct zmk_hid_led_report *report = (struct zmk_hid_led_report *)*data;
            struct zmk_endpoint_instance endpoint = {
                .transport = ZMK_TRANSPORT_USB,
            };
            zmk_hid_indicators_process_report(&report->body, endpoint);
        }
        break;
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    default:
        LOG_ERR("Invalid report ID %d requested", setup->wValue & HID_GET_REPORT_ID_MASK);
        return -EINVAL;
    }

    return 0;
}

static const struct hid_ops ops = {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    .protocol_change = set_proto_cb,
#endif
    .int_in_ready = in_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

static int zmk_usb_hid_send_report(const uint8_t *report, size_t len) {
    switch (zmk_usb_get_status()) {
    case USB_DC_SUSPEND:
        return usb_wakeup_request();
    case USB_DC_ERROR:
    case USB_DC_RESET:
    case USB_DC_DISCONNECTED:
    case USB_DC_UNKNOWN:
        return -ENODEV;
    default: {
        struct usb_hid_msg msg = {.len = len};
        memcpy(&msg.data, report, len);
        if (k_msgq_put(&usb_hid_msgq, &msg, K_NO_WAIT)) {
            LOG_ERR("failed to add HID message to queue");
        } else {
            // Add to queue. This uses "schedule" rather than "reschedule"
            // to keep the existing delay if the work item is already in the
            // queue such as following a USB HID write failure.
            k_work_schedule_for_queue(zmk_workqueue_lowprio_work_q(), &usb_hid_work, K_NO_WAIT);
        }
        return 0;
    }
    }
}

int zmk_usb_hid_send_keyboard_report(void) {
    size_t len;
    uint8_t *report = get_keyboard_report(&len);
    return zmk_usb_hid_send_report(report, len);
}

int zmk_usb_hid_send_consumer_report(void) {
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    if (hid_protocol == HID_PROTOCOL_BOOT) {
        return -ENOTSUP;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

    struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();
    return zmk_usb_hid_send_report((uint8_t *)report, sizeof(*report));
}

static int zmk_usb_hid_init(void) {
    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        LOG_ERR("Unable to locate HID device");
        return -EINVAL;
    }

    usb_hid_register_device(hid_dev, zmk_hid_report_desc, sizeof(zmk_hid_report_desc), &ops);

#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    usb_hid_set_proto_code(hid_dev, HID_BOOT_IFACE_CODE_KEYBOARD);
#endif /* IS_ENABLED(CONFIG_ZMK_USB_BOOT) */

    usb_hid_init(hid_dev);

    return 0;
}

SYS_INIT(zmk_usb_hid_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);
