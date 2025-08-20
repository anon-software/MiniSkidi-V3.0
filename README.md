* Video at [youtube.com/watch?v=dDfVdGGrf1k](https://www.youtube.com/watch?v=dDfVdGGrf1k)
* Kits at [professorboots.com/products/rc-skidsteer-kit-v3-0](https://professorboots.com/products/rc-skidsteer-kit-v3-0?utm_medium=product_shelf)
* Tutorial at [professorboots.com/a/members](https://professorboots.com/a/members) (If self sourcing the parts you'll need to pay for a membershp to unlock that specific course)
* [3D Printed RC SkidSteer V3.0 printables](https://www.printables.com/de/model/721244-3d-printed-rc-skidsteer-v30)

The project depends on these libraries:

* Async TCP by ESP32Async 3.4.7
* ESP Async WebServer by ESP32Async 3.8.0
* ESP32Servo by Kevin Harrington, John K. Bennett 3.0.5

The project depends on this boards package:

* esp32 by Espressif Systems 3.2.0

# Bluepad32 support for MiniSkidi V3.0

The game controller sketch comes from https://github.com/JohnFodero/MiniSkidi-V3.0. 

## Controls (for PS3)
 * Left X - Bucket Tilt
 * Left Y - Proportional Left Track Forward/Back
 * Right X - Arm Up/Down
 * Right Y - Proportional Right Track Foward Back
 * X - Toggle Lights

 Rumble activates when attempting to drive servos past their limit :) 
