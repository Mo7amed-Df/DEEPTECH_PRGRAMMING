#include <HardwareSerial.h>
#include <Bluepad32.h>

#define PIN_G4 4 //RS485 enabls 
#define PIN_G5 5 



 HardwareSerial MySerial(2);  // UART2
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

uint16_t buttons;
int16_t lx ;
int16_t ly ;
int16_t rx ;
int16_t ry ;
uint16_t r2;     
uint16_t l2;
uint8_t dpad;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

//SERIAEL

// void dumpGamepad(ControllerPtr ctl) {
//   Serial.printf(
//   "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//   "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
//   ctl->index(),        // Controller Index
//   ctl->dpad(),         // D-pad
//   ctl->buttons(),      // bitmask of pressed buttons
//   ctl->axisX(),        // (-511 - 512) left X Axis
//   ctl->axisY(),        // (-511 - 512) left Y axis
//   ctl->axisRX(),       // (-511 - 512) right X axis
//   ctl->axisRY(),       // (-511 - 512) right Y axis
//   ctl->brake(),        // (0 - 1023): brake button
//   ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
//   ctl->miscButtons(),  // bitmask of pressed "misc" buttons
//   ctl->gyroX(),        // Gyro X
//   ctl->gyroY(),        // Gyro Y
//   ctl->gyroZ(),        // Gyro Z
//   ctl->accelX(),       // Accelerometer X
//   ctl->accelY(),       // Accelerometer Y
//   ctl->accelZ()        // Accelerometer Z
//   );
// }

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
    buttons = ctl->buttons();
    lx = ctl->axisX();
    ly = ctl->axisY();
    rx = ctl->axisRX();
    ry = ctl->axisRY();
    l2 = ctl->brake();
    r2 =  ctl->throttle();
    dpad = ctl->dpad();
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    digitalWrite(4, HIGH);
    // code for when X button is pushed
  }
  if (ctl->buttons() != 0x0001) {
    // code for when X button is released
     digitalWrite(4, LOW);
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when square button is pushed
  }
  if (ctl->buttons() != 0x0004) {
  // code for when square button is released
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    // code for when triangle button is pushed
  }
  if (ctl->buttons() != 0x0008) {
    // code for when triangle button is released
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when circle button is pushed
  }
  if (ctl->buttons() != 0x0002) {
    // code for when circle button is released
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //== PS4 R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }

  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
  }

  //== LEFT JOYSTICK - LEFT ==//
  if (ctl->axisX() <= -25) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//
  if (ctl->axisX() >= 25) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }
  //dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setup() {
  Serial.begin(9600);  // USB Serial Monitor
 MySerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17
 // MySerial.begin(9600, SERIAL_8N1, 44, 43);
  
  delay(1000);

  pinMode(PIN_G4, OUTPUT);
  pinMode(PIN_G5, OUTPUT);
 
  pinMode(4, OUTPUT);

 // Serial.begin(115200);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

    // Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  // const uint8_t* addr = BP32.localBdAddress();
  // Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  //pinMode(48, OUTPUT);

}

void loop() {
  //MySerial.println("TEST FROM ESP32");  // Sent to STM32
 
  digitalWrite(PIN_G4, HIGH);
  digitalWrite(PIN_G5, HIGH);

  bool dataUpdated = BP32.update();
  if (dataUpdated){
    processControllers();
   //digitalWrite(48, HIGH);  // LED on

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  }
   MySerial.printf("BTN:%hu,LX:%hd,LY:%hd,RX:%hd,RY:%hd,r2:%hd,l2:%hd,dpad:%hd\n",buttons, lx, ly, rx, ry, r2, l2, dpad);

   Serial.printf("BTN:%d,LX:%d,LY:%d,RX:%d,RY:%d,r2:%d,l2:%d,dpad:%d\n", buttons, lx, ly, rx, ry,r2,l2,dpad);

  
  delay(150);
}

