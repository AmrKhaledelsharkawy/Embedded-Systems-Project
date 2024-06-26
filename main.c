#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include <string.h>  // Add this line at the top of your file


#define UART_ID uart0
#define UART_TX_PIN 164  // Set your UART TX pin number
#define UART_RX_PIN 1764  // Set your UART RX pin number
#define BAUD_RATE 9600

#define TRIG_PIN1 2
#define ECHO_PIN1 3
#define TRIG_PIN2 6
#define ECHO_PIN2 7
#define TRIG_PIN3 4
#define ECHO_PIN3 5

#define PWM_PIN1 19  // Use GPIO 14 for the first servo
#define PWM_PIN2 15  // Use GPIO 15 for the second servo

#define RED_LED1 10   // Use GPIO 10 for the red component of the first RGB LED
#define GREEN_LED1 11 // Use GPIO 11 for the green component of the first RGB LED

#define RED_LED2 12   // Use GPIO 12 for the red component of the second RGB LED
#define GREEN_LED2 13 // Use GPIO 13 for the green component of the second RGB LED

#define RED_LED3 0  // Use GPIO 12 for the red component of the second RGB LED
#define GREEN_LED3 1 // Use GPIO 13 for the green component of the second RGB LED

#define PWM_FREQ 50   // PWM frequency in Hz
#define COMM_SEND_PIN 22  // GPIO pin for sending signals to the slave
#define COMM_RECV_PIN 21  // GPIO pin for receiving signals from the slave
#define COMM_CONF_PIN 26  // GPIO pin for receiving signals from the slave
#define COMM_CONF2_PIN 17  // GPIO pin for receiving signals from the slave
#define COMM_CONF3_PIN 14  // GPIO pin for receiving signals from the slave
#define COMM_CONF4_PIN 28 // GPIO pin for receiving signals from the slave

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
void init_comm_pins() {
    gpio_init(COMM_SEND_PIN);
    gpio_set_dir(COMM_SEND_PIN, GPIO_OUT);

    gpio_init(COMM_CONF2_PIN);
    gpio_set_dir(COMM_CONF2_PIN, GPIO_OUT);


   gpio_init(Test);
    gpio_set_dir(Test, GPIO_OUT);

    gpio_init(COMM_RECV_PIN);
    gpio_set_dir(COMM_RECV_PIN, GPIO_IN);
    gpio_pull_down(COMM_RECV_PIN);  
    
    
    // Ensure it reads low when no signal is present
 
gpio_init(COMM_CONF_PIN);
    gpio_set_dir(COMM_CONF_PIN, GPIO_IN);
    gpio_pull_down(COMM_CONF_PIN);  // Ensure it reads low when no signal is present

    gpio_init(COMM_CONF3_PIN);
    gpio_set_dir(COMM_CONF3_PIN, GPIO_IN);
    gpio_pull_down(COMM_CONF3_PIN);
   gpio_init(COMM_CONF4_PIN);
    gpio_set_dir(COMM_CONF4_PIN, GPIO_OUT);
 
}

void controlRGBLed(uint redPin, uint greenPin, bool isRed, bool isGreen)
{
    gpio_put(redPin, isRed ? 1 : 0);
    gpio_put(greenPin, isGreen ? 1 : 0);
}

void test(){
   
     controlRGBLed(RED_LED1, GREEN_LED1, 1,0);
         setServoPulse(PWM_PIN1, angleToPulseWidth(0)); 
         controlRGBLed(RED_LED2, GREEN_LED2, 1,0);
         setServoPulse(PWM_PIN2, angleToPulseWidth(0));

    while(1){
    uint64_t distanceCm1 = getCm(TRIG_PIN1, ECHO_PIN1);
    uint64_t distanceCm2 = getCm(TRIG_PIN2, ECHO_PIN2);
    printf("Distance from sensor 1: %llu cm\n", distanceCm1);
    printf("Distance from sensor 2: %llu cm\n", distanceCm2);
    sleep_ms(2000);
    if(distanceCm1<20){
         controlRGBLed(RED_LED1, GREEN_LED1, 0,1);
         setServoPulse(PWM_PIN1, angleToPulseWidth(360)); // Open gate 
         // Open gate 
sleep_ms(500);}
else{
        controlRGBLed(RED_LED1, GREEN_LED1, 1,0);
         setServoPulse(PWM_PIN1, angleToPulseWidth(0)); 
}
    if(distanceCm2<20){
           controlRGBLed(RED_LED2, GREEN_LED2, 0,1);
         setServoPulse(PWM_PIN2, angleToPulseWidth(360)); // Open gate 
       
}
else{
    controlRGBLed(RED_LED2, GREEN_LED2, 1,0);
   setServoPulse(PWM_PIN2, angleToPulseWidth(0));
}}}
int main()
{
    stdio_init_all();
     uart_init1();  // Initialize UART
    init_comm_pins();

    setupUltrasonicPins(TRIG_PIN1, ECHO_PIN1);
    setupUltrasonicPins(TRIG_PIN2, ECHO_PIN2);
    setupUltrasonicPins(TRIG_PIN3, ECHO_PIN3);


    gpio_init(RED_LED1);
    gpio_set_dir(RED_LED1, GPIO_OUT);
    gpio_init(GREEN_LED1);
    gpio_set_dir(GREEN_LED1, GPIO_OUT);


    gpio_init(RED_LED2);
    gpio_set_dir(RED_LED2, GPIO_OUT);
    gpio_init(GREEN_LED2);
    gpio_set_dir(GREEN_LED2, GPIO_OUT);


    gpio_init(RED_LED3);
    gpio_set_dir(RED_LED3, GPIO_OUT);
    gpio_init(GREEN_LED3);
    gpio_set_dir(GREEN_LED3, GPIO_OUT);

    setupPWM(PWM_PIN1);
    setupPWM(PWM_PIN2);
    controlRGBLed(RED_LED1, GREEN_LED1, 1, 0);
    controlRGBLed(RED_LED2, GREEN_LED2, 1, 0);
    controlRGBLed(RED_LED3, GREEN_LED3, 1, 0);

     setServoPulse(PWM_PIN1, angleToPulseWidth(720)); // Close gate
    setServoPulse(PWM_PIN2, angleToPulseWidth(360)); // Close gate


   


while (1) {
    // ... existing code for ultrasonic sensors and UART communication ...
   uint64_t distanceCm1 = getCm(TRIG_PIN1, ECHO_PIN1);
    uint64_t distanceCm2 = getCm(TRIG_PIN2, ECHO_PIN2);

     printf("first%llu\n", distanceCm1);
     printf("secound:%llu\n", distanceCm2);
     printf("---------------------------------------");
     


     

    if (gpio_get(COMM_RECV_PIN)==1) {
      

         gpio_put(COMM_CONF2_PIN, 1);  // Send signal to slave
         controlRGBLed(RED_LED1, GREEN_LED1, 0, 1);
         setServoPulse(PWM_PIN1, angleToPulseWidth(360)); // Open gate 
          
        

    }if(distanceCm1<=20){
    //    gpio_put(COMM_CONF_PIN, 0);
          gpio_put(Test, 0);  // Send signal to slave

       controlRGBLed(RED_LED1, GREEN_LED1, 1, 0); // Set RGB LED to red
        setServoPulse(PWM_PIN1, angleToPulseWidth(720)); // Close gate
        gpio_put(COMM_CONF2_PIN, 0);  // Send signal to slave

    }

    // Send a signal to the slave Pico if a car exits
    if (distanceCm2 <= 20&& gpio_get(COMM_CONF3_PIN)==1) {
        // ... existing exit logic ...
        gpio_put(COMM_SEND_PIN, 1);
        while(1) {
               sleep_ms(500);               // Keep the signal for a short duration
            if(gpio_get(COMM_CONF_PIN)==1){
                        gpio_put(COMM_SEND_PIN, 0);
                        break;
            }
        } // Send signal to slave
        sleep_ms(500);               // Keep the signal for a short duration
         controlRGBLed(RED_LED2, GREEN_LED2, 0, 1); // Set RGB LED to green
        setServoPulse(PWM_PIN2, angleToPulseWidth(360)); // Open gate
    }
    else{
    controlRGBLed(RED_LED2, GREEN_LED2, 1, 0); // Set RGB LED to red
        setServoPulse(PWM_PIN2, angleToPulseWidth(0)); // Close gate
    }

   

    sleep_ms(500); // Delay before checking again
}

}