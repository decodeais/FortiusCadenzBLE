#include <ArduinoBLE.h>

#define CADENCE_SERVICE_UUID "1816"
#define CADENCE_CHARACTERISTIC_UUID "2A5B"

BLECharacteristic CadenceCharacteristic;
unsigned long lastPulseTime = 0;
unsigned long lastRevolutionTime=0;
int revolutions_old = 0;
unsigned long cadenceTime = 0;
unsigned long diffTime = 0;
int diffRevolutions;
int pulsCnt;
float CadFrqz = 0.0;



int revolutions = 0;
void setup() {
  lastPulseTime=millis();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D12,OUTPUT);
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
  int revolutions = 0; // Declare revolutions here

  BLEDevice cadence_periph = BLE.available();

  if (cadence_periph) {
    Serial.print("Found ");
    Serial.print(cadence_periph.address());
    Serial.print(" '");
    Serial.print(cadence_periph.localName());
    Serial.print("' ");
    Serial.print(cadence_periph.advertisedServiceUuid());
    Serial.println();

    if (cadence_periph.advertisedServiceUuid().equals(CADENCE_SERVICE_UUID)) {
      BLE.stopScan();
      Serial.println("Got cadence device, scan stopped");
      monitorSensor(cadence_periph);
      BLE.scan();
    }
  }

  delay(500);
}

void handleNotification() {
 // Serial.println("Notification received!");

  int length = CadenceCharacteristic.valueLength();
  uint8_t data[length];
  CadenceCharacteristic.readValue(data, length);

  if (length >= 3) {
    byte constantByte = data[0];
    revolutions = (data[2] << 8) | data[1];
    byte timeBits = ((data[4] & 0b00110000) >> 4) | (data[3] << 2);
    uint16_t timeInSeconds = timeBits * (1000.0 / 1024);

   /* Serial.print("Constant Byte: ");
    Serial.println(constantByte);
    Serial.print("Revolutions: ");
    Serial.println(revolutions);
    Serial.print("Time (seconds): ");
    Serial.println(timeInSeconds);*/
  }
}

void monitorSensor(BLEDevice cadence_periph) {
  static int revolutions_old=0;
  static int cadenceTime_old=0;
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
  //  while (1);
  }

  while (cadence_periph.connected()) {
    
    if (CadenceCharacteristic.valueUpdated()) {
      handleNotification();
    }
//Serial.print(revolutions);Serial.println(revolutions_old);
diffTime = millis() - lastRevolutionTime;
diffRevolutions = revolutions-revolutions_old;
    if (diffRevolutions) {
Serial.print("diffTime ");

      Serial.println(diffTime);
      Serial.println(revolutions - revolutions_old);

      cadenceTime = diffTime / (revolutions - revolutions_old);
      
     
      revolutions_old = revolutions;
      pulsCnt=0;
      lastRevolutionTime = millis();}
      else 
      {
            if (pulsCnt>2) 
            {
              cadenceTime = millis() - lastRevolutionTime;
              
            }
            //lastCandence=cadenceTime;
      }
      unsigned long pulseDiff;
      Serial.print("cadenceTime ");
      Serial.print(cadenceTime);
      if (cadenceTime!=0)    CadFrqz=CadFrqz*0.95+0.05*120000.0/(cadenceTime_old+cadenceTime);
      cadenceTime_old=cadenceTime;
Serial.print("Cadence/min ");
      Serial.println(CadFrqz);
 pulseDiff =  millis()-lastPulseTime ;
 Serial.print("pulseDiff ");
      Serial.print(pulseDiff);
    if (pulseDiff> 60000.0/CadFrqz) {
   /*  Serial.println("LED On");
      Serial.print("cadenceTime ");
      Serial.println(cadenceTime);
       Serial.print("pulseDiff ");
      Serial.println(pulseDiff);
      Serial.print("Cadence/min ");
      Serial.println(60000.0/pulseDiff);*/
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(D12, HIGH);
      lastPulseTime = millis();
      pulsCnt++;
    
    }
//Serial.println(pulseDiff);
    if (millis()-lastPulseTime > 100) {
    /*  Serial.println("LED Off");
      Serial.print("cadenceTime ");
      Serial.print(cadenceTime);
       Serial.print("pulseDiff ");
      Serial.println(pulseDiff);*/
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(D12, LOW);
    }

    
    delay(50);
  }

  Serial.println("Sensor disconnected!");
}
