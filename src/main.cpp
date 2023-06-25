#include<Arduino.h>
#include <ArduinoBLE.h>
#include<NRF52_MBED_TimerInterrupt.h>

//DEFINES
#define ANALOG_IN A3  //Pin analogico donde se realiza la lectura
#define PWR_AMP D9 //Pin alimentación amplificador
#define SW_MEDIDA D5 //Pin control SW MEDIDA
#define VREF 3.3  //Tensión de referencia del SAADC
#define N_MUESTRAS 100 //Numero de muestras que se toman para hacer la media
#define T_MUESTREO 2 //Periodo de muestreo en ms
#define T_SLEEP 2000 //Tiempo hasta hacer una nueva medida en ms
#define T_SLEEP_NC 2000 //Tiempo en ms de espera cuando no hay ningun dispositivo conectado
//void sendIndication();
float medidaIrradiancia(void);

float valor_irradiancia=0;
char buffer[16];

BLEService sensorIrradiancia("181A");  // UUID del servicio personalizado
BLEStringCharacteristic irradiancia("2A77", BLERead | BLENotify,16); // UUID de la característica personalizada

/*
//Disabling UART0 (saves around 300-500µA) - @Jul10199555 contribution
NRF_UART0->TASKS_STOPTX = 1;
NRF_UART0->TASKS_STOPRX = 1;
NRF_UART0->ENABLE = 0;

*(volatile uint32_t *)0x40002FFC = 0;
*(volatile uint32_t *)0x40002FFC;
*(volatile uint32_t *)0x40002FFC = 1; //Setting up UART registers again due to a library issue
*/

void setup() {
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH); //PIN_ENABLE_I2C_PULLUP - @pert contribution
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); //PIN_ENABLE_SENSORS_3V3 - @pert contribution

  analogReadResolution(12); //Resolucion SAADC 12bits
  nrf_saadc_disable(); //Desactivar SAADC hasta que se necesite

  // Inicializar el entorno BLE
  if (!BLE.begin()) {
    while (1);
  }

  // Configurar el servicio y la característica personalizados
  BLE.setLocalName("Nano33BLE_1");
  BLE.setAdvertisedService(sensorIrradiancia); 
  sensorIrradiancia.addCharacteristic(irradiancia);
  BLE.addService(sensorIrradiancia);
  irradiancia.setValue("0"); // Establecer el valor inicial de la característica

  // Iniciar el anuncio no conectable
  BLE.advertise();

  // Inicializar el temporizador para enviar indicaciones
  //NRF52_MBED_Timer ITimer(NRF_TIMER_3);
  //ITimer.attachInterruptInterval(5000000,sendIndication); // Enviar indicaciones cada 5 segundos
  
}

void loop() {
  BLEDevice central = BLE.central();
  if(central){
    while(central.connected()){
      //NRF_POWER->TASKS_LOWPWR=1;
      valor_irradiancia=medidaIrradiancia();
      sprintf(buffer,"%.1f",valor_irradiancia);
      irradiancia.writeValue(buffer);
      delay(T_SLEEP);
    }
  }
  delay(T_SLEEP_NC)
}

float medidaIrradiancia(void)
{
  uint32_t lectura[N_MUESTRAS];
  float lectura_aux=0, irrad_aux;

  /**
   * @brief Se activa el amplificador y el SAADC y 
   * se pone el MOSFET en saturacion para realizar una nueva medida
   */
  digitalWrite(PWR_AMP,LOW);
  digitalWrite(SW_MEDIDA,HIGH);
  delayMicroseconds(200);
  nrf_saadc_enable();
  
  /**
   * @brief Se toman N_MUESTRAS con periodo de muestreo
   * T_MUESTREO. Se hace la media de todas las medidas tomadas.
   */
  for(uint32_t i=0;i<N_MUESTRAS;i++){

    lectura[i]=analogRead(ANALOG_IN);
    lectura_aux+=lectura[i];
    delay(T_MUESTREO);
  }
  
  /**
   * @brief Se desactiva el SAADC y el amplificador para ahorrar energia
   * y se pone el MOSFET en corte para derivar la corriente del panel solar
   * hacia el BMS
   */
  nrf_saadc_disable();
  digitalWrite(PWR_AMP,HIGH);
  digitalWrite(SW_MEDIDA,LOW);

 /**
  * @brief Se calcula la irradiancia a partir de las muestras
  */
  lectura_aux=lectura_aux/N_MUESTRAS;
  irrad_aux=lectura_aux*(VREF/4095);
  return irrad_aux/10.1*1000;
}

/*void sendIndication() {
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
}*/
