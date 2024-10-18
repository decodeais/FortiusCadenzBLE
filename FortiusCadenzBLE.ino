#include <ArduinoBLE.h>

#define CADENCE_SERVICE_UUID "1816"
#define CADENCE_CHARACTERISTIC_UUID "2A5B"
#define BOOT_BUTTON_PIN 9              // Boot button (PROG button) on GPIO9
#define LED_PIN LED_BUILTIN            // Built-in LED
#define CADENCE_OUTPUT_PIN 10          // GPIO10 for cadence output

BLECharacteristic CadenceCharacteristic;
unsigned long lastPulseTime = 0;
unsigned long lastRevolutionTime = 0;
int revolutions = 0;
int revolutions_old = 0;
unsigned long cadenceTime = 0;
unsigned long diffTime = 0;
int diffRevolutions;
int pulsCnt;
float CadFrqz = 0.0;
String savedSensorAddress = "";       // To store the sensor's MAC address
unsigned long buttonPressTime = 0;
const unsigned long longPressDuration = 3000;  // 3 seconds long press to reset
bool isScanning = true;                // Variable to track scan status
unsigned long lastLedToggleTime = 0;  // Variable to manage LED blink time
const unsigned long ledBlinkInterval = 100;  // LED blinks every 100ms

void setup() {
  lastPulseTime = millis();
  pinMode(LED_PIN, OUTPUT);
  pinMode(CADENCE_OUTPUT_PIN, OUTPUT);  // Set cadence output pin as output
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // PROG button as input with pull-up
  Serial.begin(9600);
  delay(1500);
  Serial.println("start");

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  Serial.println("BLE Central");
  Serial.println("Turn on sensor and check batteries");

  BLE.scanForUuid(CADENCE_SERVICE_UUID);
}

void loop() {
  checkButtonPress();                  // Check the status of the boot button
  blinkLedDuringScan();               // Blink the LED during scanning

  BLEDevice cadence_periph = BLE.available();
  if (cadence_periph) {
    Serial.print("Found ");
    Serial.print(cadence_periph.address());
    Serial.print(" '");
    Serial.print(cadence_periph.localName());
    Serial.print("' ");
    Serial.print(cadence_periph.advertisedServiceUuid());
    Serial.println();

    String currentSensorAddress = cadence_periph.address();
    if (cadence_periph.advertisedServiceUuid().equals(CADENCE_SERVICE_UUID)) {
      BLE.stopScan();
      isScanning = false;               // Stop fast LED blinking
      digitalWrite(LED_PIN, HIGH);       // Turn off the LED

      // Save the sensor's MAC address if this is the first connection
      if (savedSensorAddress == "") {
        savedSensorAddress = currentSensorAddress;
        Serial.print("Saved sensor address: ");
        Serial.println(savedSensorAddress);
      }

      monitorSensor(cadence_periph);
      reconnectToSavedSensor();
    }
  }
}

void checkButtonPress() {
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {  // Button pressed
    if (buttonPressTime == 0) {
      buttonPressTime = millis();  // Store the time when the button was pressed
    }

    // If the button has been pressed long enough
    if (millis() - buttonPressTime >= longPressDuration) {
      Serial.println("Button pressed for 3 seconds, resetting sensor...");
      resetSensor();               // Reset the sensor
      buttonPressTime = 0;         // Reset time to capture the next press
    }
  } else {
    buttonPressTime = 0;           // Reset time
  }
}

void blinkLedDuringScan() {
  if (isScanning) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLedToggleTime >= ledBlinkInterval) {
      // Toggle the LED
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastLedToggleTime = currentMillis;  // Reset time for the next blink
    }
  } else {
    digitalWrite(LED_PIN, HIGH);  // Turn off LED when not scanning
  }
}

void resetSensor() {
  savedSensorAddress = "";          // Reset saved address
  BLE.scanForUuid(CADENCE_SERVICE_UUID);  // Start scanning for new sensors
  isScanning = true;                // Start fast LED blinking
  Serial.println("Sensor reset, scanning for new devices.");
}

void reconnectToSavedSensor() {
  if (savedSensorAddress != "") {
    Serial.println("Trying to reconnect to saved sensor...");

    BLE.scanForAddress(savedSensorAddress);

    BLEDevice savedSensor = BLE.available();
    if (savedSensor) {
      Serial.println("Reconnected to saved sensor.");
      monitorSensor(savedSensor);
      isScanning = false;           // Stop fast LED blinking
    } else {
      Serial.println("Could not reconnect to saved sensor, scanning continues.");
      isScanning = true;            // Continue scanning
    }
  }
}

void handleNotification() {
  int length = CadenceCharacteristic.valueLength();
  uint8_t data[length];
  CadenceCharacteristic.readValue(data, length);

  if (length >= 3) {
    byte constantByte = data[0];
    revolutions = (data[2] << 8) | data[1];
    byte timeBits = ((data[4] & 0b00110000) >> 4) | (data[3] << 2);
    uint16_t timeInSeconds = timeBits * (1000.0 / 1024);
  }
}

void monitorSensor(BLEDevice cadence_periph) {
  static int revolutions_old = 0;
  static int cadenceTime_old = 0;
  Serial.println("Connecting ...");

  if (cadence_periph.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  Serial.println("Discovering cadence service ...");
  if (cadence_periph.discoverService(CADENCE_SERVICE_UUID)) {
    Serial.println("Cadence Service discovered");

    CadenceCharacteristic = cadence_periph.characteristic(CADENCE_CHARACTERISTIC_UUID);

    if (CadenceCharacteristic.canSubscribe()) {
      CadenceCharacteristic.subscribe();
    } else {
      Serial.println("Cannot subscribe to notifications.");
    }
  } else {
    Serial.println("Cadence Attribute discovery failed.");
    cadence_periph.disconnect();
  }

  while (cadence_periph.connected()) {
    if (CadenceCharacteristic.valueUpdated()) {
      handleNotification();
    }

    diffTime = millis() - lastRevolutionTime;
    diffRevolutions = revolutions - revolutions_old;
    if (diffRevolutions) {
      cadenceTime = diffTime / (revolutions - revolutions_old);
      revolutions_old = revolutions;
      pulsCnt = 0;
      lastRevolutionTime = millis();

     
    } 
    else {
      if (pulsCnt > 2) {
        cadenceTime = millis() - lastRevolutionTime;
      }
    }

    unsigned long pulseDiff = millis() - lastPulseTime;
    if (cadenceTime != 0) {
      CadFrqz = CadFrqz * 0.95 + 0.05 * 120000.0 / (cadenceTime_old + cadenceTime);
      cadenceTime_old = cadenceTime;
      printf("Cadence: %3.2f \n",CadFrqz);

    }

    if (pulseDiff > 60000.0 / CadFrqz) {
      digitalWrite(LED_PIN, LOW);
       // Output cadence pulse to GPIO10
      digitalWrite(CADENCE_OUTPUT_PIN, HIGH);
      lastPulseTime = millis();
      pulsCnt++;
    }

    if (millis() - lastPulseTime > 100) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(CADENCE_OUTPUT_PIN, LOW);
    }

    delay(50);
  }

  Serial.println("Sensor disconnected!");
}
