#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // Use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define TIMER_DURATION 3500

bool blinkState = false;
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

// MPU control/status vars
bool dmpReady = false;  // Set true if DMP init was successful
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

struct TimerData {
    unsigned long startTime;
    bool timerExpired;
    unsigned long difference;
};

// Timer functions
void startTimer(TimerData& timerData) {
    Serial.println("Timer started and reset");
    timerData.startTime = millis();
    timerData.timerExpired = false;
}

void checkTimer(TimerData& timerData, unsigned long duration) {
    unsigned long elapsedTime = millis() - timerData.startTime;
    if (elapsedTime >= duration) {
        timerData.timerExpired = true;
    }
}

void resetTimer(TimerData& timerData) {
    timerData.timerExpired = false;
    timerData.startTime = millis();
}

bool isTimerExpired(const TimerData& timerData) {
    return timerData.timerExpired;
}

class detect {
public:
    float previous_value;
    int moveCounter;
    TimerData timerData;

    enum state {
        STATE_NEUTRAL,
        STATE_LEFT,
        STATE_RIGHT,
        STATE_COMPLETED
    } current_state;

    detect(unsigned long diff) : timerData{0, false, diff} {
        current_state = STATE_NEUTRAL;
        previous_value = 0.0;
        moveCounter = 0;
    }

    bool detectHeadMovement(float current_value) {
        switch (current_state) {
            case STATE_NEUTRAL:
                if (current_value >= 30) {
                    Serial.println("Moved left");
                    startTimer(timerData);
                    current_state = STATE_LEFT;
                }
                break;

            case STATE_LEFT:
                if (current_value <= -30) {
                    Serial.println("Moved right");
                    current_state = STATE_RIGHT;
                }
                break;

            case STATE_RIGHT:
                if (current_value >= -5 && current_value <= 5) {
                    Serial.println("Returned to neutral again");
                    moveCounter++;
                    if (moveCounter >= 2) {
                        current_state = STATE_COMPLETED;
                    } else {
                        current_state = STATE_NEUTRAL;
                    }
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Full movement sequence completed!");
                resetTimer(timerData);
                current_state = STATE_NEUTRAL;
                moveCounter = 0;
                return true;
        }
        return false;
    }
};

class left_detect {
public:
    float previous_value;
    TimerData timerData;

    enum state {
        STATE_NEUTRAL,
        STATE_LEFT,
        STATE_COMPLETED
    } current_state;

    left_detect(unsigned long diff) : timerData{0, false, diff} {
        current_state = STATE_NEUTRAL;
        previous_value = 0.0;
    }

    bool detectHeadMovement01(float current_value) {
        switch (current_state) {
            case STATE_NEUTRAL:
                if (current_value >= 30) {
                    Serial.println("Moved left");
                    startTimer(timerData);
                    current_state = STATE_LEFT;
                }
                break;

            case STATE_LEFT:
                if (current_value >= 5 && current_value <= 30) {
                    Serial.println("Returned to neutral position");
                    current_state = STATE_COMPLETED;
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Left movement sequence completed!");
                resetTimer(timerData);
                current_state = STATE_NEUTRAL;
                return true;
        }
        return false;
    }
};

class right_detect {
public:
    float previous_value;
    TimerData timerData;

    enum state {
        STATE_NEUTRAL,
        STATE_RIGHT,
        STATE_COMPLETED
    } current_state;

    right_detect(unsigned long diff) : timerData{0, false, diff} {
        current_state = STATE_NEUTRAL;
        previous_value = 0.0;
    }

    bool detectHeadMovement02(float current_value) {
        switch (current_state) {
            case STATE_NEUTRAL:
                if (current_value <= -30) {
                    Serial.println("Moved right");
                    startTimer(timerData);
                    current_state = STATE_RIGHT;
                }
                break;

            case STATE_RIGHT:
                if (current_value <= -5 && current_value >= -30) {
                    Serial.println("Returned to neutral");
                    current_state = STATE_COMPLETED;
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Right movement sequence completed!");
                resetTimer(timerData);
                current_state = STATE_NEUTRAL;
                return true;
        }
        return false;
    }
};
class NeutralUpNeutral {
public:
    enum State {
        STATE_NEUTRAL,
        STATE_UP,
        STATE_COMPLETED
    } currentState;

    TimerData timerData;

    NeutralUpNeutral(unsigned long diff) : timerData{0, false, diff} {
        currentState = STATE_NEUTRAL;
    }

    bool detectGesture(float pitch) {
        switch (currentState) {
            case STATE_NEUTRAL:
                if (pitch <= -30) {
                    Serial.println("Moved up");
                    startTimer(timerData);
                    currentState = STATE_UP;
                }
                break;

            case STATE_UP:
                if (pitch >= -5 && pitch<=5) {
                    Serial.println("Returned to neutral position");
                    currentState = STATE_COMPLETED;
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Neutral-up-neutral gesture detected!");
                resetTimer(timerData);
                currentState = STATE_NEUTRAL;
                return true;
        }
        return false;
    }
};

class NeutralDownNeutral {
public:
    enum State {
        STATE_NEUTRAL,
        STATE_DOWN,
        STATE_COMPLETED
    } currentState;

    TimerData timerData;

    NeutralDownNeutral(unsigned long diff) : timerData{0, false, diff} {
        currentState = STATE_NEUTRAL;
    }

    bool detectGesture(float pitch) {
        switch (currentState) {
            case STATE_NEUTRAL:
                if (pitch <= 30) {
                    Serial.println("Moved down");
                    startTimer(timerData);
                    currentState = STATE_DOWN;
                }
                break;

            case STATE_DOWN:
                if (pitch <= 5 && pitch >=-5) {
                    Serial.println("Returned to neutral position");
                    currentState = STATE_COMPLETED;
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Neutral-down-neutral gesture detected!");
                resetTimer(timerData);
                currentState = STATE_NEUTRAL;
                return true;
        }
        return false;
    }
};

class NeutralUpDownUpDownNeutral {
public:
    enum State {
        STATE_NEUTRAL,
        STATE_UP,
        STATE_DOWN,
        STATE_SECOND_UP,
        STATE_SECOND_DOWN,
        STATE_COMPLETED
    } currentState;

    TimerData timerData;

    NeutralUpDownUpDownNeutral(unsigned long diff) : timerData{0, false, diff} {
        currentState = STATE_NEUTRAL;
    }

    bool detectGesture(float pitch) {
        switch (currentState) {
            case STATE_NEUTRAL:
                if (pitch <= -30) {
                    Serial.println("Moved up");
                    startTimer(timerData);
                    currentState = STATE_UP;
                }
                break;

            case STATE_UP:
                if (pitch >= 30) {
                    Serial.println("Moved down");
                    currentState = STATE_DOWN;
                }
                break;

            case STATE_DOWN:
                if (pitch <= -30 ) {
                    Serial.println("Moved up again");
                    currentState = STATE_SECOND_UP;
                }
                break;

            case STATE_SECOND_UP:
                if (pitch >= 30) {
                    Serial.println("Moved down again");
                    currentState = STATE_SECOND_DOWN;
                }
                break;

            case STATE_SECOND_DOWN:
                if (pitch >= -5 && pitch <= 5) {
                    Serial.println("Returned to neutral");
                    currentState = STATE_COMPLETED;
                }
                break;

            case STATE_COMPLETED:
                Serial.println("Neutral-up-down-up-down-neutral gesture detected!");
                resetTimer(timerData);
                currentState = STATE_NEUTRAL;
                return true;
        }
        return false;
    }
};


// Define objects
detect obj(600);
left_detect obj1(600);
right_detect obj2(600);
NeutralUpNeutral gesture1(600);
NeutralDownNeutral gesture2(600);
NeutralUpDownUpDownNeutral gesture3(600);


// Interrupt handler
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Send any character to begin DMP programming and demo: "));
    while (!Serial.available());
    while (Serial.available() && Serial.read());

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    if (!dmpReady) return;

    if (!mpuInterrupt && !mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);


    obj.detectHeadMovement(ypr[0] * 180 / M_PI);
    obj1.detectHeadMovement01(ypr[0] * 180 / M_PI);
    obj2.detectHeadMovement02(ypr[0] * 180 / M_PI);
    gesture1.detectGesture(ypr[2] * 180 / M_PI);  // Pitch for Neutral-Up-Neutral
    gesture2.detectGesture(ypr[2] * 180 / M_PI);  // Pitch for Neutral-Down-Neutral
    gesture3.detectGesture(ypr[2] * 180 / M_PI);

    if (isTimerExpired(obj.timerData)) {
        Serial.println("Movement timed out");
        resetTimer(obj.timerData);
    }

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(1000);
}
