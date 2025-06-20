/*
    Bluepad32 x MiniSkidi_3.0
*/
#include <Arduino.h>

#include <ESP32Servo.h> // by Kevin Harrington
#include <vector>
#include <Bluepad32.h>

// defines
#define bucketServoPin  23
#define auxServoPin 22
#define lightPin1 18
#define lightPin2 5

#define FORWARD 1
#define BACKWARD -1

#define DEAD_ZONE 20

#define MOTOR_PWM_FREQ 5000
#define MOTOR_PWM_RES 8

#define DEBOUNCE_MILLIS 300

struct Measurement {
  int average = 0, countAverage = 0;
  int last = 0, countLast = 0;
  void reset() {
    average = 0; countAverage = 0;
    last = 0; countLast = 0;
  }
};
struct Calibration {
  Measurement
    axisX,
    axisY,
    axisRX,
    axisRY,
    brake,
    throttle;
  void reset() {
    axisX.reset();
    axisY.reset();
    axisRX.reset();
    axisRY.reset();
    brake.reset();
    throttle.reset();
  }
} calibration;

struct Throttle {
  unsigned long lastStart;
  unsigned long lastDuration;
  bool run(unsigned long duration) {
    unsigned long current = millis();
    if (current - lastStart >= lastDuration) {
      lastStart = current;
      lastDuration = duration;
      return true;
    }
    return false;
  }
};

Throttle rumbleThrottle;
void rumble(ControllerPtr gamepad, int duration) {
  if (rumbleThrottle.run(duration))
    gamepad->playDualRumble(0,         // delayedStartMs,
                            duration,  // durationMs,
                            0x80,      // weakMagnitude,
                            0xc0       // strongMagnitude
    );
}

struct ThrottledServo: Servo {
  Throttle servoThrottle;
  int servo_pos = 0;
  int lo, hi;
  int minThrottle, maxThrottle;
  int maxInput;
  int deadZone;
  void servoMove(ControllerPtr gamepad, int servoValue);
  ThrottledServo(int servo_pos, int lo, int hi, int minThrottle, int maxThrottle, int maxInput, int deadZone):
    servo_pos(servo_pos),
    lo(lo),
    hi(hi),
    minThrottle(minThrottle),
    maxThrottle(maxThrottle),
    maxInput(maxInput),
    deadZone(deadZone) {
  }
  void init(int pin) {
    int channel = attach(pin);
    Serial.printf("Attached %d %d\n", channel, servo_pos);
    write(servo_pos);
  }
};

ThrottledServo bucketServo(170, 10, 175, 8, 30, 511, 70); // Grabs PWM Channel 0
ThrottledServo auxServo(150, 90, 170, 12, 30, 1023, 70); // Grabs PWM Channel 1

bool light = false;
int light_debounce_ms = millis();
int now = light_debounce_ms;

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
  int channel1;
  int channel2;
};

std::vector<MOTOR_PINS> motorPins =
{
  {25, 26, 2, 3},  //RIGHT_MOTOR Pins
  {33, 32, 4, 5},  //LEFT_MOTOR Pins
  {21, 19, 6, 7},  //ARM_MOTOR Pins
};

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
void ThrottledServo::servoMove(ControllerPtr gamepad, int servoValue) {
  if (abs(servoValue) <= deadZone)
    return;
  servoValue = constrain(servoValue, -maxInput, maxInput);

  int x = map(abs(servoValue), 0, maxInput, maxThrottle, minThrottle);
  if (servoThrottle.run(x)) {
    int new_pos = servo_pos + sgn(servoValue);
    Serial.printf("Servo %d %d %d %d %lu\n", servoValue, x, servo_pos, new_pos, millis());
    if (new_pos > hi | new_pos < lo) {
      rumble(gamepad, 1000/*ms*/);
    }
    else if (new_pos != servo_pos) {
      servo_pos = new_pos;
      Serial.printf("servo_pos %d\n", servo_pos);
      write(servo_pos);
    }
  }
}
void bucketTilt(ControllerPtr gamepad, int bucketServoValue)
{
  bucketServo.servoMove(gamepad, bucketServoValue);
}
void auxControl(ControllerPtr gamepad, int auxServoValue)
{
  auxServo.servoMove(gamepad, auxServoValue);
}
void lightControl()
{
  now = millis();
  if (now - light_debounce_ms > DEBOUNCE_MILLIS) {
    light_debounce_ms = now;
    if (!light)
    {
      digitalWrite(lightPin1, HIGH);
      digitalWrite(lightPin2, LOW);
      light = true;
    }
    else
    {
      digitalWrite(lightPin1, LOW);
      digitalWrite(lightPin2, LOW);
      light = false;
    }

  }

}

void setUpPinModes()
{

  for (int i = 0; i < motorPins.size(); i++)
  {
    ledcSetup(motorPins[i].channel1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(motorPins[i].pinIN1, motorPins[i].channel1);
    ledcWrite(motorPins[i].channel1, 0);

    ledcSetup(motorPins[i].channel2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(motorPins[i].pinIN2, motorPins[i].channel2);
    ledcWrite(motorPins[i].channel2, 0);
  }

  bucketServo.init(bucketServoPin);
  auxServo.init(auxServoPin);

  pinMode(lightPin1, OUTPUT);
  pinMode(lightPin2, OUTPUT);
}

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial port to connect.
  }

  setUpPinModes();
  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void stopAllMotors() {
  for (int i = 0; i < motorPins.size(); i++)
  {
    ledcWrite(motorPins[i].channel1, 0);
    ledcWrite(motorPins[i].channel2, 0);
  }
}

void calibrateOne(Measurement &m, int t) {
  if (m.last == t)
    m.countLast++;
  else
    m.countLast = 0;
  m.last = t;
  if (m.countLast >= 100) {
    long x = ((long)m.countLast * (long)m.last + (long)m.countAverage * (long)m.average) / ((long)m.countLast + (long)m.countAverage);
    m.average = x;
    m.countAverage += m.countLast;
    m.countLast = 0;
  }
}
void calibrateAll(ControllerPtr gamepad) {
  calibrateOne(calibration.axisX, gamepad->axisX());
  calibrateOne(calibration.axisY, gamepad->axisY());
  calibrateOne(calibration.axisRX, gamepad->axisRX());
  calibrateOne(calibration.axisRY, gamepad->axisRY());
  calibrateOne(calibration.brake, gamepad->brake());
  calibrateOne(calibration.throttle, gamepad->throttle());
  printf(
          "Calibration axis L: %4li-%4li, %4li-%4li, axis R: %4li-%4li, %4li-%4li, "
          "brake: %4ld-%4li, throttle: %4li-%4li\n",
          calibration.axisX.average,
          calibration.axisX.last,
          calibration.axisY.average,
          calibration.axisY.last,
          calibration.axisRX.average,
          calibration.axisRX.last,
          calibration.axisRY.average,
          calibration.axisRY.last,
          calibration.brake.average,
          calibration.brake.last,
          calibration.throttle.average,
          calibration.throttle.last
    );
}

void processGamepad(ControllerPtr gamepad) {
  static bool calibrationInProgress = false;
  if (calibrationInProgress) {
    calibrateAll(gamepad);
  }
  else {
    // Set motor speeds
    int left_yaxis = gamepad->axisY();
    if (abs(left_yaxis) > DEAD_ZONE) {
      // update left motor speed
      int mapped_ly = map(left_yaxis, -512, 512, -255, 255);
      if (mapped_ly < 0) {
        ledcWrite(motorPins[1].channel1, abs(mapped_ly));
        ledcWrite(motorPins[1].channel2, 0);
      } else {
        ledcWrite(motorPins[1].channel1, 0);
        ledcWrite(motorPins[1].channel2, mapped_ly);
      }
    } else {
      // stop motors
      ledcWrite(motorPins[1].channel1, 0);
      ledcWrite(motorPins[1].channel2, 0);
    }

    int right_yaxis = gamepad->axisRY();
    if (abs(right_yaxis) > DEAD_ZONE) {
      // update right motor speed
      int mapped_ry = map(right_yaxis, -512, 512, -255, 255);
      if (mapped_ry < 0) {
        ledcWrite(motorPins[0].channel1, abs(mapped_ry));
        ledcWrite(motorPins[0].channel2, 0);
      } else {
        ledcWrite(motorPins[0].channel1, 0);
        ledcWrite(motorPins[0].channel2, mapped_ry);
      }
    } else {
      // stop motors
      ledcWrite(motorPins[0].channel1, 0);
      ledcWrite(motorPins[0].channel2, 0);
    }
    // set arm motor
    int right_xaxis = gamepad->axisRX();
    if (abs(right_xaxis) > DEAD_ZONE) {
      int mapped_rx = map(right_xaxis, -512, 512, -255, 255);
      if (mapped_rx < 0) {
        ledcWrite(motorPins[2].channel1, 0);
        ledcWrite(motorPins[2].channel2, abs(mapped_rx));
      } else {
        ledcWrite(motorPins[2].channel1, mapped_rx);
        ledcWrite(motorPins[2].channel2, 0);
      }
    } else {
      // stop motors
      ledcWrite(motorPins[2].channel1, 0);
      ledcWrite(motorPins[2].channel2, 0);
    }
    // set bucket servo
    int left_xaxis = gamepad->axisX() - calibration.axisX.average;
    bucketTilt(gamepad, left_xaxis);
    // set claw servo
    int claw_raw = gamepad->throttle() - gamepad->brake();
    auxControl(gamepad, claw_raw);
    // Set lights
    if (gamepad->a()) {
      lightControl();
    }

    // Rumble the controller
    if (gamepad->x()) {
      rumble(gamepad, 1000/*ms*/);
    }
    char buf[256];
    snprintf(buf, sizeof(buf) - 1,
            "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
            "axis L: %4li, %4li, axis R: %4li, %4li, "
            "brake: %4ld, throttle: %4li, misc: 0x%02x, "
            "gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
            "battery: %d",
            gamepad->index(),        // Gamepad Index
            gamepad->dpad(),         // DPAD
            gamepad->buttons(),      // bitmask of pressed buttons
            gamepad->axisX(),        // (-511 - 512) left X Axis
            gamepad->axisY(),        // (-511 - 512) left Y axis
            gamepad->axisRX(),       // (-511 - 512) right X axis
            gamepad->axisRY(),       // (-511 - 512) right Y axis
            gamepad->brake(),        // (0 - 1023): brake button
            gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
            gamepad->miscButtons(),  // bitmak of pressed "misc" buttons
            gamepad->gyroX(),      // Gyro X
            gamepad->gyroY(),      // Gyro Y
            gamepad->gyroZ(),      // Gyro Z
            gamepad->accelX(),     // Accelerometer X
            gamepad->accelY(),     // Accelerometer Y
            gamepad->accelZ(),     // Accelerometer Z
            gamepad->battery()       // 0=Unknown, 1=empty, 255=full
    );
    Serial.println(buf);
  }
  if (gamepad->y()) {
    calibrationInProgress = !calibrationInProgress;
    if (calibrationInProgress)
      calibration.reset();
    delay(DEBOUNCE_MILLIS);
  }
}

void loop() {
  BP32.update();

  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    } else {
      stopAllMotors();
    }
  }
  vTaskDelay(1);
}
