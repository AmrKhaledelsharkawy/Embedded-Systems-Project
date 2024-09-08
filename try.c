#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include <string.h>

// UART settings
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUD_RATE 9600

// Ultrasonic sensor pins
#define TRIG_PIN1 2
#define ECHO_PIN1 3
#define TRIG_PIN2 6
#define ECHO_PIN2 7

// Servo motor pins
#define PWM_PIN1 19  // Servo 1
#define PWM_PIN2 15  // Servo 2

// RGB LED pins
#define RED_LED1 10   // RGB LED 1 Red
#define GREEN_LED1 11 // RGB LED 1 Green
#define RED_LED2 12   // RGB LED 2 Red
#define GREEN_LED2 13 // RGB LED 2 Green

// PWM frequency for servos
#define PWM_FREQ 50
#define UART_BUFFER_SIZE 128

// UART buffer
char uart_buffer[UART_BUFFER_SIZE];
int uart_buffer_index = 0;

// Timeout for ultrasonic sensor
int timeout = 26100;

// Function prototypes
void setupUltrasonicPins(uint trigPin, uint echoPin);
uint64_t getPulse(uint trigPin, uint echoPin);
uint64_t getCm(uint trigPin, uint echoPin);
void setupPWM(uint pwmPin);
void controlServo(uint pwmPin, float angle);
void controlRGBLed(uint redPin, uint greenPin, bool isRed, bool isGreen);
void uart_init1(void);

int main() {
    stdio_init_all();
    uart_init1();

    // Setup ultrasonic sensors
    setupUltrasonicPins(TRIG_PIN1, ECHO_PIN1);
    setupUltrasonicPins(TRIG_PIN2, ECHO_PIN2);

    // Setup servos
    setupPWM(PWM_PIN1);
    setupPWM(PWM_PIN2);

    // Setup LEDs
    gpio_init(RED_LED1);
    gpio_set_dir(RED_LED1, GPIO_OUT);
    gpio_init(GREEN_LED1);
    gpio_set_dir(GREEN_LED1, GPIO_OUT);

    gpio_init(RED_LED2);
    gpio_set_dir(RED_LED2, GPIO_OUT);
    gpio_init(GREEN_LED2);
    gpio_set_dir(GREEN_LED2, GPIO_OUT);

    while (1) {
        // Get distances from both ultrasonic sensors
        uint64_t distance1 = getCm(TRIG_PIN1, ECHO_PIN1);
        uint64_t distance2 = getCm(TRIG_PIN2, ECHO_PIN2);

        // Print distances to UART
        printf("Distance 1: %llu cm, Distance 2: %llu cm\n", distance1, distance2);

        // Control servos based on distance (e.g., move to 90 degrees if distance < 10 cm)
        if (distance1 < 10) {
            controlServo(PWM_PIN1, 90);  // Move servo 1 to 90 degrees
        } else {
            controlServo(PWM_PIN1, 0);   // Move servo 1 to 0 degrees
        }

        if (distance2 < 10) {
            controlServo(PWM_PIN2, 90);  // Move servo 2 to 90 degrees
        } else {
            controlServo(PWM_PIN2, 0);   // Move servo 2 to 0 degrees
        }

        // Control RGB LEDs based on distance
        if (distance1 < 10) {
            controlRGBLed(RED_LED1, GREEN_LED1, true, false);  // Turn red on, green off
        } else {
            controlRGBLed(RED_LED1, GREEN_LED1, false, true);  // Turn green on, red off
        }

        if (distance2 < 10) {
            controlRGBLed(RED_LED2, GREEN_LED2, true, false);  // Turn red on, green off
        } else {
            controlRGBLed(RED_LED2, GREEN_LED2, false, true);  // Turn green on, red off
        }

        sleep_ms(1000);  // Wait 1 second before next measurement
    }

    return 0;
}

// Setup ultrasonic sensor pins
void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

// Get pulse duration from ultrasonic sensor
uint64_t getPulse(uint trigPin, uint echoPin) {
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0)
        tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        width++;
        sleep_us(1);
        if (width > timeout)
            return 0;
    }
    absolute_time_t endTime = get_absolute_time();

    return absolute_time_diff_us(startTime, endTime);
}

// Convert pulse duration to distance in centimeters
uint64_t getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

// Setup PWM for servos
void setupPWM(uint pwmPin) {
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pwmPin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_init(slice_num, &config, true);
}

// Control servo angle
void controlServo(uint pwmPin, float angle) {
    uint slice_num = pwm_gpio_to_slice_num(pwmPin);
    float duty_cycle = (angle / 180.0) * 100.0 + 5.0;  // Convert angle to duty cycle
    pwm_set_gpio_level(pwmPin, (uint) duty_cycle);
}

// Control RGB LEDs
void controlRGBLed(uint redPin, uint greenPin, bool isRed, bool isGreen) {
    gpio_put(redPin, isRed ? 1 : 0);
    gpio_put(greenPin, isGreen ? 1 : 0);
}

// Initialize UART
void uart_init1() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}
