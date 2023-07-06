/**
 * @file main.cpp
 * @author David Hergueta Soto
 * @brief Código del sensor de irradiancia del TFG. Este código está preparado para
 * funcionar sobre un Arduino Nano 33 BLE
 *  
 * @version 1.0
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "fsm.h"

//MACROS
#define ANALOG_IN A3  //Pin analogico donde se realiza la lectura
#define PWR_AMP D9 //Pin alimentación amplificador
#define SW_MEDIDA D5 //Pin control SW MEDIDA
#define VREF 3.3  //Tensión de referencia del SAADC
#define N_MUESTRAS 100 //Numero de muestras que se toman para hacer la media
#define T_MUESTREO 2 //Periodo de muestreo en ms
#define T_SLEEP 2000 //Tiempo hasta hacer una nueva medida en ms
#define T_SLEEP_NC 1000 //Tiempo en ms de espera cuando no hay ningun dispositivo conectado
#define T_POLLING 500 //Tiempo en ms del tiempo que se encuentra haciendo polling
#define T_ADVERTISING 16000 //Tiempo en ms de intervalos entre advertising

//MANEJADORES INTERRUPCIÓN
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);

//ESTADOS FSM
enum states{
  ESPERAR_CONEXION,
  MUESTREO,
  FIN_MEDIDA
};

//VARIABLES GLOBALES
float lectura = 0;
volatile uint8_t connected=0;
uint32_t next=0;
uint8_t N=0;
fsm_t* fsm_sensor = NULL;

//OTRAS FUNCIONES PARA LA FSM
void begin(BLEService* servicio, BLECharacteristic* caracteristica, BLEDeviceEventHandler handler_connection,
 BLEDeviceEventHandler handler_disconnection);
void stopMedida(void);
void startMedida(void);
void delayUntil(uint32_t* ultima_activacion, uint32_t tiempo_delay);
void publicarMedida(float medida, BLEStringCharacteristic* caracteristica);
uint32_t now(void);

//FUNCIONES DE GUARDA. SE EJECUTAN CUANDO SE HACE EFECTIVA UNA TRANSICIÓN
static void prepararMuestreo(fsm_t* f);
static void pollConnection(fsm_t* f);
static void terminarMuestreo(fsm_t* f);
static void muestrear(fsm_t* f);

//FUNCIONES DE TRANSICIÓN. CONTROLAN LA REALIZACIÓN DE TRANSICIONES EN LA FSM
static int conectado(fsm_t* f){if(connected&&(now()>=next))return 1; else return 0;}
static int desconectado(fsm_t* f){if((!connected)&&(now()>=next))return 1; else return 0;}
static int muestreoFinalizado(fsm_t* f){if(N>=N_MUESTRAS)return 1; else return 0;}
static int muestreoNoFinalizado(fsm_t* f){if(N<N_MUESTRAS&&(now()>=next))return 1; else return 0;}


//TABLA DE TRANSICIONES
static fsm_trans_t sensor_tt[]={
  {ESPERAR_CONEXION, conectado, MUESTREO, prepararMuestreo},
  {ESPERAR_CONEXION, desconectado, ESPERAR_CONEXION, pollConnection},
  {MUESTREO,muestreoFinalizado,FIN_MEDIDA,terminarMuestreo},
  {MUESTREO,muestreoNoFinalizado,MUESTREO,muestrear},
  {FIN_MEDIDA,desconectado,ESPERAR_CONEXION,pollConnection},
  {FIN_MEDIDA, conectado, MUESTREO, prepararMuestreo},
  {-1, NULL, -1, NULL},
};

//SERVICIO Y CARACTERÍSTICA BLE
BLEService sensorIrradiancia("181A");  // UUID del servicio personalizado
BLEStringCharacteristic irradiancia("2A77", BLERead | BLENotify,16); // UUID de la característica personalizada



void setup() {
  begin(&sensorIrradiancia, &irradiancia,blePeripheralConnectHandler,blePeripheralDisconnectHandler);
  stopMedida();
  fsm_sensor = fsm_new(sensor_tt);
  next=millis();
}

void loop() {
  fsm_fire(fsm_sensor);
}

//DEFINICION FUNCIONES DE GUARDA
static void prepararMuestreo(fsm_t* f)
{
  N=0;
  lectura=0;
  startMedida();
  next=now();
}

static void pollConnection(fsm_t* f)
{
  BLE.poll(T_POLLING);
  delayUntil(&next,T_SLEEP_NC);
}

static void terminarMuestreo(fsm_t* f)
{
  stopMedida();
  lectura=lectura*(VREF/4095);
  publicarMedida(lectura, &irradiancia);
  BLE.poll(T_POLLING);
  delayUntil(&next,T_SLEEP);
}

static void muestrear(fsm_t* f)
{
  lectura+=analogRead(ANALOG_IN)/N_MUESTRAS;
  N++;
  delayUntil(&next,T_MUESTREO);
}

//DEFINICIÓN DE OTRAS FUNCIONES
void blePeripheralConnectHandler(BLEDevice central)
{
  connected=1;
}
void blePeripheralDisconnectHandler(BLEDevice central)
{
  connected=0;
}

/**
 * @brief Inicializa todos los perfiféricos necesarios
 * y realiza la configuración de la conexión BLE
 */
void begin(BLEService* servicio, BLECharacteristic* caracteristica, BLEDeviceEventHandler handler_connection,
 BLEDeviceEventHandler handler_disconnection)
{
//Desactivar UART0
  NRF_UART0->TASKS_STOPTX = 1;
  NRF_UART0->TASKS_STOPRX = 1;
  NRF_UART0->ENABLE = 0;

  *(volatile uint32_t *)0x40002FFC = 0;
  *(volatile uint32_t *)0x40002FFC;
  *(volatile uint32_t *)0x40002FFC = 1;
  
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW); //APAGAR LED DE ENCENDIDO
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW); //PIN_ENABLE_SENSORS_3V3
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW); //PIN_ENABLE_I2C_PULLUP

  analogReadResolution(12); //Resolucion SAADC 12bits

  // Inicializar el entorno BLE
  if (!BLE.begin()) {
    while (1);
  }

  // Configurar el servicio y la característica personalizados
  BLE.setLocalName("Nano33BLE_1");

  BLE.setAdvertisedService(*servicio); 
  BLE.setAdvertisingInterval(T_ADVERTISING);
  
  sensorIrradiancia.addCharacteristic(*caracteristica);
  BLE.addService(*servicio);
  caracteristica->setValue("0"); // Establecer el valor inicial de la característica

  //Event handlers para la conexión y desconexión
  BLE.setEventHandler(BLEConnected, handler_connection);
  BLE.setEventHandler(BLEDisconnected, handler_disconnection);

  // Iniciar el anuncio no conectable
  BLE.advertise();
}

/**
 * @brief Pone el MOSFET en corte y apaga el SAADC y el amplificador
 * para reducir consumo y recargar la bateria cuando no se necesite medir
 */
void stopMedida(void)
{
  digitalWrite(PWR_AMP,HIGH); //Desactivar MCP6023
  digitalWrite(SW_MEDIDA,LOW); //Poner MOSFET en corte
  nrf_saadc_disable(); //Desactivar SAADC hasta que se necesite
}

/**
 * @brief Pone el MOSFET en saturación y enciende el SAADC y el amplificador
 * para comenzar a realizar medidas
 */
void startMedida(void)
{
  digitalWrite(PWR_AMP,LOW); //Activar MCP6023
  digitalWrite(SW_MEDIDA,HIGH); //MOSFET en saturación
  nrf_saadc_enable(); //Activar SAADC
  delay(1);
}

/**
 * @brief Espera hasta el tiempo necesario hasta que pase el tiempo fijado en tiempo_delay
 * desde la ultima activacion almacenada en ultima_activacion.
 * tiempo_delay debe estar en ms
 *
 * @param ultima_activacion 
 * @param tiempo_delay 
 */
void delayUntil(uint32_t* ultima_activacion, uint32_t tiempo_delay)
{
  uint32_t current_time=millis();
  if(current_time<(*ultima_activacion+tiempo_delay)){
    delay(*ultima_activacion+tiempo_delay-current_time);
  }
  *ultima_activacion+=tiempo_delay;
}

/**
 * @brief Escribe el valor de medida como string en la característica que se le haya pasado
 * 
 * @param medida 
 * @param caracteristica 
 */
void publicarMedida(float medida, BLEStringCharacteristic* caracteristica)
{
  char buffer[16];

  sprintf(buffer,"%.1f",medida);
  caracteristica->writeValue(buffer);
}

/**
 * @brief Obtiene el tiempo transcurrido desde que empezó el programa en ms
 * 
 * @return uint32_t 
 */
uint32_t now(void)
{
  return millis();
}