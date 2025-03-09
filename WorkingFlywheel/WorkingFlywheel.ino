const int potPin = A0;        // Potentiometer connection
const int pwmPin = 9;         // Enable Pin to L298
const int beginPin = 7;       // ATTACH PIN 7 to host Arduino
const int hallSensorPin = 2;  // Hall effect digital output

volatile unsigned long lastTriggerTime = 0; // Stores last detection time
volatile unsigned long hallInterval = 1000; // Time between hall triggers (in µs)
const int stallRecover = 255;


int pwmValue = stallRecover;           // PWM output value
float targetFrequency = 80;    // Target speed (Hz)
float actualFrequency = 0;    // Measured frequency

int pwmFeedforward = 0;
const float Kf = 2.2;         // Experimentally determined

// PID control variables
float Kp = 3;  // Proportional gain set to 9 for grey wheel
float Ki = 0.0;  // Integral gain set to 0.1 for grey wheel
float Kd = 0.00; // Derivative gain

float previousError = 0;
float integral = 0;

void setup() {
    pinMode(pwmPin, OUTPUT);
    pinMode(beginPin, INPUT);
    pinMode(hallSensorPin, INPUT);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallTriggered, FALLING);
    analogWrite(pwmPin, 0);
}

void loop() {
    unsigned long currentMicros = micros();
    targetFrequency = 80;
    if (digitalRead(beginPin)) {
      if (hallInterval > 0 && currentMicros - lastTriggerTime > 200000) {  // .2 seconds without hall trigger
            pwmValue = stallRecover;
      } else {
        // Convert hall interval to freq
        if (hallInterval > 0) {
            actualFrequency = 1e6 / hallInterval;  // Convert µs interval to Hz
        } else { // catch division by 0
            actualFrequency = 0;
        }
        float error = targetFrequency - actualFrequency;
        integral += error;
        pwmValue = Kp * error + Ki * integral;
        pwmFeedforward = Kf * targetFrequency; 
        pwmValue += pwmFeedforward;
        pwmValue = constrain(pwmValue, 0, 255);
        previousError = error;
      }
    } else {
      pwmValue = 0;
    }
    Serial.print(digitalRead(beginPin));
    Serial.print("   ");
    Serial.println(pwmValue);
    analogWrite(pwmPin, pwmValue);
}

void hallTriggered() {
    unsigned long currentTime = micros();
    if (currentTime - lastTriggerTime > 4000) {  // 4 ms debounce
        hallInterval = currentTime - lastTriggerTime;
        lastTriggerTime = currentTime;
    }
}
