#include<Arduino.h>
#include <ArduinoBLE.h>
#include<NRF52_MBED_TimerInterrupt.h>

#define LED_PWR 25

void sendIndication();

BLEService customService("19B10000-E8F2-537E-4F6C-D104768A1214");  // UUID del servicio personalizado
BLEUnsignedCharCharacteristic customCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // UUID de la característica personalizada

void setup() {
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW);
  Serial.begin(9600);

  // Inicializar el entorno BLE
  if (!BLE.begin()) {
    Serial.println("No se pudo iniciar BLE");
    while (1);
  }

  // Configurar el servicio y la característica personalizados
  BLE.setLocalName("Nano33BLE");
  BLE.setAdvertisedService(customService); 
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.setValue(0); // Establecer el valor inicial de la característica

  // Iniciar el anuncio no conectable
  BLE.advertise();
  Serial.println("Esperando conexión BLE...");

  // Inicializar el temporizador para enviar indicaciones
  NRF52_MBED_Timer ITimer(NRF_TIMER_3);
  ITimer.attachInterruptInterval(5000000,sendIndication); // Enviar indicaciones cada 5 segundos
  
}

void loop() {
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  // Esperar eventos BLE
  BLE.poll();
}

void sendIndication() {
  digitalWrite(LED_PWR,!digitalRead(LED_PWR));
  // Obtener el valor actual de la característica
  uint8_t value = customCharacteristic.value();

  // Incrementar el valor en 1
  value++;

  // Establecer el nuevo valor de la característica
  customCharacteristic.writeValue(value);

  // Enviar la indicación a los dispositivos conectados
  BLEDevice central = BLE.central();
  if (central) {
    customCharacteristic.broadcast();
    Serial.print("Enviando indicación: ");
    Serial.println(value);
  }
}
