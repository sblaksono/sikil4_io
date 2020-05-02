
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <wiringPi.h>
#include "SPIdev.h"
#include "sikil4_io/adc.h"
#include "sikil4_io/pwm.h"

#define LED_BLUE    5
#define LED_GREEN   6

#define CMD_PWM     0x10
#define CMD_ADC     0x80

void ledBlueCallback(const std_msgs::BoolConstPtr& msg)
{
    digitalWrite(LED_BLUE, msg->data ? LOW : HIGH) ; 
}

void ledGreenCallback(const std_msgs::BoolConstPtr& msg)
{
    digitalWrite(LED_GREEN, msg->data ? LOW : HIGH) ; 
}

void writePWM(uint8_t ch, uint16_t data) {
    unsigned char tx[3] = {CMD_PWM + ch, data >> 8, data & 0xFF};
    unsigned char rx[3] = {0, 0, 0};

    SPIdev::transfer("/dev/spidev1.0", tx, rx, 3);
}

uint16_t readADC(uint8_t ch) {
    unsigned char tx[3] = {CMD_ADC + ch, 0, 0};
    unsigned char rx[3] = {0, 0, 0};

    SPIdev::transfer("/dev/spidev1.0", tx, rx, 3);
    return (rx[1] << 8) | rx[2];
}

void pwmCallback(const sikil4_io::pwmConstPtr& pwm)
{
    writePWM(pwm->channel, pwm->value);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sikil4_io");

    // prepare leds
    wiringPiSetup();
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_BLUE, HIGH) ; 
    digitalWrite(LED_GREEN, HIGH) ; 

    ros::NodeHandle nh;
    ros::Subscriber pwm_sub = nh.subscribe("pwm", 100, pwmCallback);
    ros::Subscriber led_blue_sub = nh.subscribe("leds/blue", 100, ledBlueCallback);
    ros::Subscriber led_green_sub = nh.subscribe("leds/green", 100, ledGreenCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // turn off leds
    digitalWrite(LED_BLUE, HIGH) ; 
    digitalWrite(LED_GREEN, HIGH) ; 

    return 0;
}
