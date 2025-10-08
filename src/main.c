#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

// Pin Mapping
#define X_PIN_ANALOG 1  // AIN2 for X-axis
#define Y_PIN_ANALOG 2  // AIN1 for Y-axis

// Digital pins
#define JOYSTICK_BUTTON_PIN 10
static const int button_pins[] = {3, 4, 5, 6, 7, 8};
#define NUM_BUTTONS ARRAY_SIZE(button_pins)

static const struct device *gpio_dev;
static const struct device *adc_dev;

// USB HID Device
static const struct device *hid_dev;
static bool hid_ready = false;

// ADC buffers
static int16_t adc_buffer[2];
static struct adc_sequence sequence = {
    .channels = BIT(0) | BIT(1),
    .buffer = adc_buffer,
    .buffer_size = sizeof(adc_buffer),
    .resolution = 12,
};

// Standard Gamepad HID Report Descriptor
static const uint8_t hid_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    
    // Buttons (8 buttons - 1 byte)
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        // Usage Minimum (Button 1)
    0x29, 0x08,        // Usage Maximum (Button 8)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data,Var,Abs)
    
    // X and Y axes (8-bit each)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x15, 0x00,        // Logical Minimum (0)
    0x26, 0xFF, 0x00,  // Logical Maximum (255)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x02,        // Report Count (2)
    0x81, 0x02,        // Input (Data,Var,Abs)
    
    0xC0,              // End Collection
};

// HID Report Structure for 8-bit axes
struct hid_joystick_report {
    uint8_t buttons;
    uint8_t x;
    uint8_t y;
} __packed;

static struct hid_joystick_report joystick_report;

// USB Status Callback
static void hid_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status) {
    case USB_DC_ERROR:
        printk("USB error occurred\n");
        hid_ready = false;
        break;
    case USB_DC_RESET:
        printk("USB reset occurred\n");
        break;
    case USB_DC_CONNECTED:
        printk("USB connected\n");
        break;
    case USB_DC_CONFIGURED:
        printk("USB configured - HID ready!\n");
        hid_ready = true;
        break;
    case USB_DC_DISCONNECTED:
        printk("USB disconnected\n");
        hid_ready = false;
        break;
    case USB_DC_SUSPEND:
        printk("USB suspended\n");
        break;
    case USB_DC_RESUME:
        printk("USB resumed\n");
        break;
    case USB_DC_UNKNOWN:
    default:
        printk("USB unknown state: %d\n", status);
        break;
    }
}

void setup_adc(void) {
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return;
    }

    // Configure X-axis (AIN2)
    struct adc_channel_cfg channel_cfg_x = {
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + X_PIN_ANALOG,
    };

    // Configure Y-axis (AIN1)  
    struct adc_channel_cfg channel_cfg_y = {
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 1,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + Y_PIN_ANALOG,
    };

    adc_channel_setup(adc_dev, &channel_cfg_x);
    adc_channel_setup(adc_dev, &channel_cfg_y);
}

int convert_to_10bit_range(int value_12bit) {
    return (value_12bit * 1023) / 4095;
}

void setup_usb_hid(void) {
    // Initialize USB
    int ret = usb_enable(hid_status_cb);
    if (ret != 0) {
        printk("Failed to enable USB: %d\n", ret);
        return;
    }
    
    // Get HID device
    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        printk("Cannot get HID device\n");
        return;
    }

    // Register HID device
    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), NULL);

    // Initialize HID
    usb_hid_init(hid_dev);
    
    printk("USB HID initialized, waiting for host...\n");
}

void update_joystick_report(int x_value, int y_value, int joystick_button) {
    // Clear previous button states
    joystick_report.buttons = 0;
    
    // Button 1: Joystick press
    if (joystick_button == 0) {
        joystick_report.buttons |= 0x01;
    }
    
    // Buttons 2-7: Digital buttons
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (gpio_pin_get(gpio_dev, button_pins[i]) == 0) {
            joystick_report.buttons |= (1 << (i + 1));
        }
    }
    
    // Convert to 8-bit range and INVERT Y-axis
    joystick_report.x = (uint8_t)((x_value * 255) / 1023);
    joystick_report.y = (uint8_t)(255 - ((y_value * 255) / 1023));  // Inverted Y
}

bool send_hid_report(void) {
    if (!hid_ready || hid_dev == NULL) {
        return false;
    }
    
    int ret = hid_int_ep_write(hid_dev, (uint8_t*)&joystick_report, 
                              sizeof(joystick_report), NULL);
    if (ret < 0) {
        if (ret != -EAGAIN) {
            printk("HID write failed: %d\n", ret);
        }
        return false;
    }
    
    return true;
}

void print_debug_info(int x_value, int y_value, int joystick_button) {
    static int count = 0;
    
    if (count % 20 == 0) {
        printk("\nX\tY\tButtons\tBinary\tHID Ready\n");
        printk("---\t---\t-------\t------\t---------\n");
    }
    
    // Convert to 8-bit for display
    uint8_t x_8bit = (x_value * 255) / 1023;
    uint8_t y_8bit = 255 - ((y_value * 255) / 1023);  // Inverted for display
    
    printk("%d\t%d\t", x_8bit, y_8bit);
    
    // Print button states
    uint8_t buttons = 0;
    if (joystick_button == 0) buttons |= 0x01;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (gpio_pin_get(gpio_dev, button_pins[i]) == 0) {
            buttons |= (1 << (i + 1));
        }
    }
    
    printk("0x%02X\t", buttons);
    
    // Print binary representation
    for (int i = 7; i >= 0; i--) {
        printk("%d", (buttons >> i) & 1);
    }
    
    printk("\t%s", hid_ready ? "YES" : "NO");
    printk("\n");
    
    count++;
}

int main(void) {
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return 0;
    }

    // Configure buttons
    gpio_pin_configure(gpio_dev, JOYSTICK_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_pin_configure(gpio_dev, button_pins[i], GPIO_INPUT | GPIO_PULL_UP);
    }

    setup_adc();
    setup_usb_hid();

    printk("=== USB HID JOYSTICK READY ===\n");
    printk("Y-axis inverted fix applied\n");
    printk("Waiting for USB configuration...\n");
    printk("====================================\n\n");

    k_msleep(1000);

    while (1) {
        if (adc_read(adc_dev, &sequence) == 0) {
            int x_value = convert_to_10bit_range(adc_buffer[0]);
            int y_value = convert_to_10bit_range(adc_buffer[1]);
            int joystick_button = gpio_pin_get(gpio_dev, JOYSTICK_BUTTON_PIN);

            // Update HID report (with Y-axis inversion)
            update_joystick_report(x_value, y_value, joystick_button);
            
            // Send HID report only if ready
            if (hid_ready) {
                send_hid_report();
            }
            
            // Print debug info
            static int debug_count = 0;
            if (debug_count % 10 == 0) {
                print_debug_info(x_value, y_value, joystick_button);
            }
            debug_count++;
        }

        k_msleep(20);
    }

    return 0;
}