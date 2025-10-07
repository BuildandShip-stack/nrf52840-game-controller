// // #include <zephyr/kernel.h>
// // #include <zephyr/sys/printk.h>
// // #include <zephyr/drivers/gpio.h>
// // #include <zephyr/drivers/adc.h>
// // #include <zephyr/devicetree.h>

// // // Pin definitions - Arduino shield mapping
// // #define JOYSTICK_X_ANALOG_PIN  1  // AIN1 for A0
// // #define JOYSTICK_Y_ANALOG_PIN  2  // AIN2 for A1
// // #define JOYSTICK_BUTTON_PIN   10  // P1.10 for D8

// // // Button pins D2-D7 on GPIO1
// // static const int button_pins[] = {3, 4, 5, 6, 7, 8};
// // static const int num_buttons = 6;

// // static const struct device *gpio_dev;
// // static const struct device *adc_dev;

// // // ADC channel IDs
// // #define X_ADC_CHANNEL 0
// // #define Y_ADC_CHANNEL 1

// // // Advanced filtering settings
// // #define SAMPLE_COUNT 16     // Number of samples for moving average
// // #define DEADZONE 15         // Ignore small changes around center

// // // Moving average buffers
// // static int x_samples[SAMPLE_COUNT] = {0};
// // static int y_samples[SAMPLE_COUNT] = {0};
// // static int sample_index = 0;

// // // Calibrated center positions (adjust these based on your readings)
// // static int x_center = 115;  // Adjust based on your center X reading
// // static int y_center = 700;  // Adjust based on your center Y reading

// // // Simple absolute value function
// // static int abs_val(int value)
// // {
// //     return (value < 0) ? -value : value;
// // }

// // // Initialize ADC
// // int init_adc(void)
// // {
// //     adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    
// //     if (!device_is_ready(adc_dev)) {
// //         printk("ADC device not ready\n");
// //         return -1;
// //     }
    
// //     printk("ADC initialized successfully\n");
// //     return 0;
// // }

// // // Read single ADC channel with proper setup
// // int read_adc_channel(uint8_t channel_id, uint8_t analog_pin)
// // {
// //     int16_t adc_value;
// //     int ret;

// //     // Configure ADC channel
// //     struct adc_channel_cfg channel_cfg = {
// //         .gain = ADC_GAIN_1_4,
// //         .reference = ADC_REF_INTERNAL,
// //         .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
// //         .channel_id = channel_id,
// //         .input_positive = analog_pin,
// //     };

// //     ret = adc_channel_setup(adc_dev, &channel_cfg);
// //     if (ret != 0) {
// //         return -1;
// //     }

// //     // Configure sequence
// //     struct adc_sequence sequence = {
// //         .channels = BIT(channel_id),
// //         .buffer = &adc_value,
// //         .buffer_size = sizeof(adc_value),
// //         .resolution = 12,
// //     };

// //     ret = adc_read(adc_dev, &sequence);
// //     if (ret != 0) {
// //         return -1;
// //     }

// //     return (int)adc_value;
// // }

// // // Convert 12-bit ADC value to 10-bit Arduino range
// // int convert_to_arduino_range(int value_12bit)
// // {
// //     if (value_12bit < 0) return -1;
// //     return (value_12bit * 1023) / 4095;
// // }

// // // Moving average filter
// // void apply_moving_average(int x_raw, int y_raw, int *x_filtered, int *y_filtered)
// // {
// //     // Add new samples to buffers
// //     x_samples[sample_index] = x_raw;
// //     y_samples[sample_index] = y_raw;
    
// //     // Move to next position in circular buffer
// //     sample_index = (sample_index + 1) % SAMPLE_COUNT;
    
// //     // Calculate moving average
// //     int x_sum = 0, y_sum = 0;
// //     for (int i = 0; i < SAMPLE_COUNT; i++) {
// //         x_sum += x_samples[i];
// //         y_sum += y_samples[i];
// //     }
    
// //     *x_filtered = x_sum / SAMPLE_COUNT;
// //     *y_filtered = y_sum / SAMPLE_COUNT;
// // }

// // // Apply deadzone around center
// // void apply_deadzone(int *x, int *y)
// // {
// //     if (abs_val(*x - x_center) < DEADZONE) {
// //         *x = x_center;
// //     }
// //     if (abs_val(*y - y_center) < DEADZONE) {
// //         *y = y_center;
// //     }
// // }

// // // Read and filter joystick values
// // void read_joystick_filtered(int *x_value, int *y_value)
// // {
// //     int x_raw, y_raw;
// //     int x_avg, y_avg;
    
// //     // Read X-axis
// //     x_raw = read_adc_channel(X_ADC_CHANNEL, JOYSTICK_X_ANALOG_PIN);
// //     if (x_raw < 0) {
// //         *x_value = -1;
// //         *y_value = -1;
// //         return;
// //     }
    
// //     // Read Y-axis
// //     y_raw = read_adc_channel(Y_ADC_CHANNEL, JOYSTICK_Y_ANALOG_PIN);
// //     if (y_raw < 0) {
// //         *x_value = -1;
// //         *y_value = -1;
// //         return;
// //     }
    
// //     // Convert to Arduino range
// //     x_raw = convert_to_arduino_range(x_raw);
// //     y_raw = convert_to_arduino_range(y_raw);
    
// //     // Apply moving average filter
// //     apply_moving_average(x_raw, y_raw, &x_avg, &y_avg);
    
// //     // Apply deadzone
// //     apply_deadzone(&x_avg, &y_avg);
    
// //     *x_value = x_avg;
// //     *y_value = y_avg;
// // }

// // // Initialize GPIO for buttons
// // int init_gpio(void)
// // {
// //     int ret;
    
// //     gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    
// //     if (!device_is_ready(gpio_dev)) {
// //         printk("GPIO device not ready\n");
// //         return -1;
// //     }

// //     // Configure joystick button
// //     ret = gpio_pin_configure(gpio_dev, JOYSTICK_BUTTON_PIN, 
// //                             GPIO_INPUT | GPIO_PULL_UP);
// //     if (ret != 0) {
// //         printk("Failed to configure joystick button: %d\n", ret);
// //         return ret;
// //     }

// //     // Configure all other buttons
// //     for (int i = 0; i < num_buttons; i++) {
// //         ret = gpio_pin_configure(gpio_dev, button_pins[i], 
// //                                 GPIO_INPUT | GPIO_PULL_UP);
// //         if (ret != 0) {
// //             printk("Failed to configure button D%d: %d\n", i + 2, ret);
// //             return ret;
// //         }
// //     }
    
// //     printk("GPIO initialized successfully\n");
// //     return 0;
// // }

// // // Read button states
// // void read_buttons(int *joystick_button, bool *button_states)
// // {
// //     // Read joystick button (active low)
// //     *joystick_button = gpio_pin_get(gpio_dev, JOYSTICK_BUTTON_PIN);
    
// //     // Read all other buttons
// //     for (int i = 0; i < num_buttons; i++) {
// //         button_states[i] = (gpio_pin_get(gpio_dev, button_pins[i]) == 0);
// //     }
// // }

// // // Auto-calibrate center position
// // void auto_calibrate(void)
// // {
// //     printk("Auto-calibrating... Keep joystick centered!\n");
    
// //     int x_sum = 0, y_sum = 0;
// //     const int cal_samples = 50;
    
// //     for (int i = 0; i < cal_samples; i++) {
// //         int x_raw = read_adc_channel(X_ADC_CHANNEL, JOYSTICK_X_ANALOG_PIN);
// //         int y_raw = read_adc_channel(Y_ADC_CHANNEL, JOYSTICK_Y_ANALOG_PIN);
        
// //         if (x_raw >= 0 && y_raw >= 0) {
// //             x_sum += convert_to_arduino_range(x_raw);
// //             y_sum += convert_to_arduino_range(y_raw);
// //         }
// //         k_msleep(10);
// //     }
    
// //     x_center = x_sum / cal_samples;
// //     y_center = y_sum / cal_samples;
    
// //     printk("Calibration complete: X_center=%d, Y_center=%d\n", x_center, y_center);
// // }

// // void main(void)
// // {
// //     printk("=== nRF52840 Funduino Joystick Test ===\n");
// //     printk("Initializing peripherals...\n");

// //     // Initialize ADC
// //     if (init_adc() != 0) {
// //         return;
// //     }

// //     // Initialize GPIO
// //     if (init_gpio() != 0) {
// //         return;
// //     }

// //     // Auto-calibrate center position
// //     auto_calibrate();

// //     printk("\nPin Mapping:\n");
// //     printk("Joystick X: A0 -> AIN1 (P0.03)\n");
// //     printk("Joystick Y: A1 -> AIN2 (P0.04)\n");
// //     printk("Buttons: D2-D7 -> P1.03-P1.08\n");
// //     printk("Joy Button: D8 -> P1.10\n");
// //     printk("Filter: %d-sample moving average, %d deadzone\n", SAMPLE_COUNT, DEADZONE);
// //     printk("Calibration: X=%d, Y=%d\n", x_center, y_center);
// //     printk("==============================\n\n");

// //     // Initialize sample buffers with center values
// //     for (int i = 0; i < SAMPLE_COUNT; i++) {
// //         x_samples[i] = x_center;
// //         y_samples[i] = y_center;
// //     }

// //     int x_value, y_value;
// //     int joystick_button;
// //     bool button_states[num_buttons];
// //     uint32_t counter = 0;

// //     while (1) {
// //         // Read joystick with advanced filtering
// //         read_joystick_filtered(&x_value, &y_value);
        
// //         // Read buttons
// //         read_buttons(&joystick_button, button_states);

// //         // Print status every 10 iterations
// //         if (counter % 10 == 0) {
// //             if (x_value >= 0 && y_value >= 0) {
// //                 printk("Joystick X: %4d, Y: %4d | Button: %s", 
// //                        x_value, y_value,
// //                        joystick_button == 0 ? "PRESSED " : "Released");
// //             } else {
// //                 printk("Joystick X: ----, Y: ---- | Button: %s",
// //                        joystick_button == 0 ? "PRESSED " : "Released");
// //             }

// //             // Check if any buttons are pressed
// //             bool any_pressed = false;
// //             for (int i = 0; i < num_buttons; i++) {
// //                 if (button_states[i]) {
// //                     if (!any_pressed) {
// //                         printk(" | Buttons: ");
// //                         any_pressed = true;
// //                     }
// //                     printk("D%d ", i + 2);
// //                 }
// //             }
            
// //             if (any_pressed) {
// //                 printk("PRESSED");
// //             }
            
// //             printk("\n");
// //         }

// //         counter++;
// //         k_msleep(50); // Faster sampling for better filtering
// //     }
// // }



//working 
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>

// CORRECT PIN MAPPING:
#define X_PIN_ANALOG 2  // AIN2 for X-axis (CONFIRMED)
#define Y_PIN_ANALOG 1  // AIN1 for Y-axis (CONFIRMED)

// Digital pins
#define JOYSTICK_BUTTON_PIN 10
static const int button_pins[] = {3, 4, 5, 6, 7, 8};
#define NUM_BUTTONS ARRAY_SIZE(button_pins)

static const struct device *gpio_dev;
static const struct device *adc_dev;

static int16_t adc_buffer[2];
static struct adc_sequence sequence = {
    .channels = BIT(0) | BIT(1),
    .buffer = adc_buffer,
    .buffer_size = sizeof(adc_buffer),
    .resolution = 12,
};

void setup_adc(void) {
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return;
    }

    // Configure X-axis (AIN2)
    struct adc_channel_cfg channel_cfg_x = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + X_PIN_ANALOG, // AIN2
    };

    // Configure Y-axis (AIN1)  
    struct adc_channel_cfg channel_cfg_y = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 1,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + Y_PIN_ANALOG, // AIN1
    };

    adc_channel_setup(adc_dev, &channel_cfg_x);
    adc_channel_setup(adc_dev, &channel_cfg_y);
}

int convert_to_arduino_range(int value_12bit) {
    return (value_12bit * 1023) / 4095;
}

void main(void) {
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return;
    }

    // Configure buttons
    gpio_pin_configure(gpio_dev, JOYSTICK_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_pin_configure(gpio_dev, button_pins[i], GPIO_INPUT | GPIO_PULL_UP);
    }

    setup_adc();

    printk("=== JOYSTICK WORKING - CORRECT PINS ===\n");
    printk("X-axis: AIN%d | Y-axis: AIN%d\n", X_PIN_ANALOG, Y_PIN_ANALOG);
    printk("Expected behavior:\n");
    printk("  Center: X=512, Y=512\n");
    printk("  Left:   X=0,   Y=512\n");
    printk("  Right:  X=1023,Y=512\n");
    printk("  Up:     X=512, Y=0\n");
    printk("  Down:   X=512, Y=1023\n");
    printk("=======================================\n\n");

    while (1) {
        if (adc_read(adc_dev, &sequence) == 0) {
            int x_value = convert_to_arduino_range(adc_buffer[0]);
            int y_value = convert_to_arduino_range(adc_buffer[1]);
            
            int joystick_button = gpio_pin_get(gpio_dev, JOYSTICK_BUTTON_PIN);

            printk("X: %4d | Y: %4d | Btn: %s",
                   x_value, y_value,
                   joystick_button == 0 ? "Pressed" : "Released");

            for (int i = 0; i < NUM_BUTTONS; i++) {
                if (gpio_pin_get(gpio_dev, button_pins[i]) == 0) {
                    printk(" | D%d:Pressed", i + 2);
                }
            }

            printk("\n");
        }

        k_msleep(50);
    }
}