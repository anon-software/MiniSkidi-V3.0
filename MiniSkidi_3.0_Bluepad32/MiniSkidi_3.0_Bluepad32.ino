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
#define RUBLE_MILLIS 1000

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

struct Debouncer {
  std::string name;
  unsigned long last = 0;
  unsigned long debounce;
  bool (Controller::*fptr)() const;
  bool press(ControllerPtr gamepad) {
    unsigned long now = millis();
    if (now - last > debounce) {
      if ((gamepad->*fptr)()) {
        last = now;
        if (name.length()>0)
          Serial.printf("button %s\n", name.c_str());
        return true;
      }
    }
    return false;
  }
  Debouncer(bool (Controller::*fptr)() const, long debounce = DEBOUNCE_MILLIS): fptr(fptr), debounce(debounce) {}
  Debouncer(const std::string &name, bool (Controller::*fptr)() const, long debounce = DEBOUNCE_MILLIS): Debouncer(fptr, debounce) {
    this->name = name;
  }
};

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
Debouncer x("rumble", &Controller::x);
Debouncer b("control", &Controller::b);
Debouncer a("light", &Controller::a);

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
      rumble(gamepad, RUBLE_MILLIS);
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

/*
Traditionl control:
Left X - Bucket Tilt
Left Y - Proportional Left Track Forward/Back
Right X - Arm Up/Down
Right Y - Proportional Right Track Foward Back

Simplified control:
Left X - Move Forward/Backward
Left Y - Turn Left/Right
Right X - Arm Up/Down
Right Y - Bucket Tilt
*/
bool traditionalControl = true;

void propulsionControl(ControllerPtr gamepad, int &left_motor, int &right_motor) {
  if (traditionalControl) {
    left_motor = gamepad->axisY() - calibration.axisY.average;
    right_motor = gamepad->axisRY() - calibration.axisRY.average;
    if (abs(left_motor) <= DEAD_ZONE)
      left_motor = 0;
    if (abs(right_motor) <= DEAD_ZONE)
      right_motor = 0;
  }
  else {
    int speed = gamepad->axisY() - calibration.axisY.average;
    if (abs(speed) <= DEAD_ZONE)
      speed = 0;
    int steer = gamepad->axisX() - calibration.axisX.average;
    if (abs(steer) <= DEAD_ZONE)
      steer = 0;
    left_motor = speed - steer;
    left_motor = constrain(left_motor, -512, 512);
    right_motor = speed + steer;
    right_motor = constrain(right_motor, -512, 512);
  }
}

int armControl(ControllerPtr gamepad) {
  int arm_motor = gamepad->axisRX() - calibration.axisRX.average;
  if (abs(arm_motor) <= DEAD_ZONE)
    arm_motor = 0;
  return arm_motor;
}

int bucketControl(ControllerPtr gamepad) {
  return traditionalControl? gamepad->axisX() - calibration.axisX.average: gamepad->axisRY() - calibration.axisRY.average;
}

void processGamepad(ControllerPtr gamepad) {
  static bool calibrationInProgress = false;
  if (calibrationInProgress) {
    calibrateAll(gamepad);
  }
  else {
    // Set motor speeds
    int left_motor;
    int right_motor;
    propulsionControl(gamepad, left_motor, right_motor);

    // update left motor speed
    int mapped_left = map(left_motor, -512, 512, -255, 255);
    if (mapped_left < 0) {
      ledcWrite(motorPins[1].channel1, abs(mapped_left));
      ledcWrite(motorPins[1].channel2, 0);
    } else {
      ledcWrite(motorPins[1].channel1, 0);
      ledcWrite(motorPins[1].channel2, mapped_left);
    }

    // update right motor speed
    int mapped_right = map(right_motor, -512, 512, -255, 255);
    if (mapped_right < 0) {
      ledcWrite(motorPins[0].channel1, abs(mapped_right));
      ledcWrite(motorPins[0].channel2, 0);
    } else {
      ledcWrite(motorPins[0].channel1, 0);
      ledcWrite(motorPins[0].channel2, mapped_right);
    }

    // set arm motor
    int arm_motor = armControl(gamepad);
    int mapped_arm = map(arm_motor, -512, 512, -255, 255);
    if (mapped_arm < 0) {
      ledcWrite(motorPins[2].channel1, 0);
      ledcWrite(motorPins[2].channel2, abs(mapped_arm));
    } else {
      ledcWrite(motorPins[2].channel1, mapped_arm);
      ledcWrite(motorPins[2].channel2, 0);
    }
    // set bucket servo
    int bucket_move = bucketControl(gamepad) - calibration.axisX.average;
    bucketTilt(gamepad, bucket_move);
    // set claw servo
    int claw_move = gamepad->throttle() - gamepad->brake();
    auxControl(gamepad, claw_move);
    // Set lights
    if (a.press(gamepad)) {
      lightControl();
    }

    // Rumble the controller
    if (x.press(gamepad)) {
      rumble(gamepad, RUBLE_MILLIS);
    }

    // Toggle traditional/simplified control
    if (b.press(gamepad)) {
      traditionalControl = !traditionalControl;
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
