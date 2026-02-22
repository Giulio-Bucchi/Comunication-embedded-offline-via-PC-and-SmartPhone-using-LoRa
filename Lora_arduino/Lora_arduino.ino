#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

const int csPin = 10;          
const int resetPin = 9;        
const int irqPin = 2;
SoftwareSerial ble(8, 7); 

#define ADDR_ARDUINO 0x02
#define ADDR_STM32   0x01
#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK  0x02

String inputBLE = "";
String lastSentMsg = "";

unsigned long lastBleRead = 0;
const int bleTimeout = 50;

void setup() {
  Serial.begin(115200); 
  ble.begin(9600);
  
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(433E6)) {
    Serial.println("Errore LoRa!");
    while (1);
  }
  LoRa.setSyncWord(0xF3);
  Serial.println("--- SISTEMA PULITO: TEST INVIO SINGOLO ---");
}


void loop() {
  // 1.(BLE)
  while (ble.available() > 0) {
    char c = (char)ble.read();
    
    if (c == '\n' || c == '\r') {
      processAndSend();
      break;
    }
    
    inputBLE += c;
    lastBleRead = millis();

    if (inputBLE.length() >= 4) {
        int mid = inputBLE.length() / 2;
        String firstHalf = inputBLE.substring(0, mid);
        String secondHalf = inputBLE.substring(mid);
        if (firstHalf == secondHalf) {
            inputBLE = firstHalf; // Remove duplicated part
        }
    }
  }

  if (inputBLE.length() > 0 && (millis() - lastBleRead > 100)) {
    processAndSend();
  }

  checkLoRa();
}

void processAndSend() {
  inputBLE.trim();
  
  // if the message is the same as the last sent one and it was sent less than 500ms ago, ignore it.
  // This is to prevent sending the same message multiple times due to BLE duplicates.
  if (inputBLE.length() > 0 && inputBLE != lastSentMsg) {
    Serial.print("Invio a STM32 (Filtrato): ");
    Serial.println(inputBLE);
    
    sendLoRaPacket(inputBLE);
    lastSentMsg = inputBLE; 
  }
  
  inputBLE = ""; 
  while(ble.available()) ble.read(); 
}

void sendLoRaPacket(String text) {
  static uint8_t seq_ctr = 0;
  
  LoRa.beginPacket();
  LoRa.write(ADDR_STM32);
  LoRa.write(ADDR_ARDUINO);
  LoRa.write(PKT_TYPE_DATA);
  LoRa.write(seq_ctr++);
  LoRa.write((uint8_t)text.length());
  LoRa.print(text);
  LoRa.endPacket();
  
  delay(30); // Radio "breathing" time
}

void checkLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    uint8_t buf[255];
    int idx = 0;
    while (LoRa.available() && idx < 255) {
      buf[idx++] = LoRa.read();
    }

    if (idx >= 5 && buf[0] == ADDR_ARDUINO && buf[2] == PKT_TYPE_DATA) {
      int len = buf[4];
      Serial.print("Da STM32: ");
      for(int i=0; i<len; i++) {
        char c = (char)buf[5+i];
        Serial.print(c);
        ble.print(c); // Send to iPhone
      }
      Serial.println();
      ble.println(); // Newline for iPhone
    }
  }
}