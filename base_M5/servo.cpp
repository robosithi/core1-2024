#include "servo.hh"

/**
 * Servo Module settings
*/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVOMIN \
    102  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX \
    512  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN \
    500  // This is the rounded 'minimum' microsecond length based on the
#define USMAX \
    2500  // This is the rounded 'maximum' microsecond length based on the
#define SERVO_FREQ \
    50  // Analog servos run at ~50 Hz updates  模拟伺服以 ~50 Hz 更新运行


void setServoPulse(uint8_t n, double pulse) {
    double pulselength;
    pulselength = 1000000;  // 1,000,000 us per second
    pulselength /= 50;      // 50 Hz
    // Serial.print(pulselength);
    // Serial.println(" us per period");
    pulselength /= 4096;  // 12 bits of resolution
    // Serial.print(pulselength);
    // Serial.println(" us per bit");
    pulse *= 1000;
    pulse /= pulselength;
    // Serial.println(pulse);
    pwm.setPWM(n, 0, pulse);
}

void servo_angle_write(uint8_t n, int Angle) {
    double pulse = Angle;
    pulse        = pulse / 90 + 0.5;
    setServoPulse(n, pulse);
}
