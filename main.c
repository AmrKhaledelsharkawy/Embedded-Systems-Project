#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include <string.h>

#define UART_ID uart0
#define UART_TX_PIN 16  // Set your UART TX pin number
#define UART_RX_PIN 17  // Set your UART RX pin number
#define BAUD_RATE 9600

#define TRIG_PIN1 2
#define ECHO_PIN1 3
#define TRIG_PIN2 6
#define ECHO_PIN2 7
#define TRIG_PIN3 4
#define ECHO_PIN3 5

#define PWM_PIN1 19  // Use GPIO 19 for the first servo
#define PWM_PIN2 15  // Use GPIO 15 for the second servo

#define RED_LED1 0   // Corrected GPIO for first RGB red LED
#define GREEN_LED1 1 // Corrected GPIO for first RGB green LED

#define RED_LED2 12   // Corrected GPIO for second RGB red LED
#define GREEN_LED2 13 // Corrected GPIO for second RGB green LED

#define PWM_FREQ 50   // PWM frequency in Hz
#define Test 16

#define UART_BUFFER_SIZE 128
char uart_buffer[UART_BUFFER_SIZE];
int uart_buffer_index = 0;
int timeout = 26100;

void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

uint64_t getPulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;
    while (gpio_get(echoPin) == 0)
        tight_loop_contents();

    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1)
    {
        width++;
        sleep_us(1);
        if (width > timeout)
            return 0;
    }
    absolute_time_t endTime = get_absolute_time();

    return absolute_time_diff_us(startTime, endTime);
}

uint64_t getCm(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    return pulseLength / 29 / 2;
}

void setupPWM(uint pwmPin)
{
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pwmPin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_init(slice_num, &config, true);
}

void controlServo(uint pwmPin, float angle)
{
    uint slice_num = pwm_gpio_to_slice_num(pwmPin);
    float duty_cycle = (angle / 180.0) * 100.0 + 5.0;  // Convert angle to duty cycle
    pwm_set_gpio_level(pwmPin, (uint) duty_cycle);
}

void setServoPulse(uint gpio, uint16_t width)
{
    pwm_set_gpio_level(gpio, width);
}

uint16_t angleToPulseWidth(int angle)
{
    const int pulseWidthMin = 1000;
    const int pulseWidthMax = 2000;
    return pulseWidthMin + (angle * (pulseWidthMax - pulseWidthMin) / 180);
}

void uart_init1() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void controlRGBLed(uint redPin, uint greenPin, bool isRed, bool isGreen)
{
    gpio_put(redPin, isRed ? 1 : 0);
    gpio_put(greenPin, isGreen ? 1 : 0);
}

int main()
{
    stdio_init_all();
    uart_init1();

    // Initialize the RGB LEDs
    gpio_init(RED_LED1);
    gpio_set_dir(RED_LED1, GPIO_OUT);
    gpio_init(GREEN_LED1);
    gpio_set_dir(GREEN_LED1, GPIO_OUT);

    gpio_init(RED_LED2);
    gpio_set_dir(RED_LED2, GPIO_OUT);
    gpio_init(GREEN_LED2);
    gpio_set_dir(GREEN_LED2, GPIO_OUT);

    // Test RGB LED control
    controlRGBLed(RED_LED1, GREEN_LED1, true, false);  // Turn RED LED1 on, Green LED1 off
    sleep_ms(1000);  // Delay 1 second
    controlRGBLed(RED_LED1, GREEN_LED1, false, true);  // Turn RED LED1 off, Green LED1 on
    sleep_ms(1000);  // Delay 1 second

    controlRGBLed(RED_LED2, GREEN_LED2, true, false);  // Turn RED LED2 on, Green LED2 off
    sleep_ms(1000);  // Delay 1 second
    controlRGBLed(RED_LED2, GREEN_LED2, false, true);  // Turn RED LED2 off, Green LED2 on
    sleep_ms(1000);  // Delay 1 second

    return 0;
}
