/*
 * TEST ESP32 to OBDII

// Retorna el estado de los monitores de diagnóstico OBD
uint32_t monitorStatus();
Bit 0: Monitores de misfires (fallos de encendido).
Bit 1: Sistema de combustible.
Bit 2: Monitores del sistema de componentes.
Bit 3: Monitores de catalizadores.
Bit 4: Monitores de sensores de oxígeno.
Bit 5: Monitores de evaporación.
Bit 6 y siguientes: Otros monitores específicos dependiendo del vehículo y el estándar OBD-II.
// Retorna el código de diagnóstico (DTC) más reciente
uint16_t freezeDTC();
// Retorna el estado del sistema de combustible
uint16_t fuelSystemStatus();
// Retorna la carga del motor en porcentaje (%)
float engineLoad();
// Retorna la temperatura del refrigerante del motor en grados Celsius
float engineCoolantTemp();
// Retorna el ajuste a corto plazo de combustible en el banco 1 (en porcentaje)
float shortTermFuelTrimBank_1();
// Retorna el ajuste a largo plazo de combustible en el banco 1 (en porcentaje)
float longTermFuelTrimBank_1();
// Retorna el ajuste a corto plazo de combustible en el banco 2 (en porcentaje)
float shortTermFuelTrimBank_2();
// Retorna el ajuste a largo plazo de combustible en el banco 2 (en porcentaje)
float longTermFuelTrimBank_2();
// Retorna la presión de combustible en kPa
float fuelPressure();
// Retorna la presión del colector de admisión en kPa
uint8_t manifoldPressure();
// Retorna las revoluciones por minuto (RPM) del motor
float rpm();
// Retorna la velocidad del vehículo en kilómetros por hora (km/h)
int32_t kph();
// Retorna la velocidad del vehículo en millas por hora (mph)
float mph();
// Retorna el avance de encendido en grados antes del punto muerto superior (PMS)
float timingAdvance();
// Retorna la temperatura del aire de admisión en grados Celsius
float intakeAirTemp();
// Retorna la tasa de flujo de masa de aire (MAF) en gramos por segundo
float mafRate();
// Retorna la posición del acelerador en porcentaje (%)
float throttle();
// Retorna el estado de la segunda válvula de aire secundaria (0-3, dependiendo del estado)
uint8_t commandedSecAirStatus();
// Retorna los sensores de oxígeno presentes en los dos primeros bancos
uint8_t oxygenSensorsPresent_2banks();
// Retorna los estándares OBD soportados por el vehículo
uint8_t obdStandards();
// Retorna los sensores de oxígeno presentes en los cuatro bancos
uint8_t oxygenSensorsPresent_4banks();
// Retorna el estado de la entrada auxiliar (true si está activa, false si no)
bool auxInputStatus();
// Retorna el tiempo de operación del motor en segundos
uint16_t runTime();
*/
#define ARDUHAL_LOG_LEVEL_NONE  //ARDUHAL_LOG_LEVEL_ERROR

#include "BluetoothSerial.h"  // Incluimos la biblioteca para manejar comunicación Bluetooth Serial
#include "ELMduino.h"         // Incluimos la biblioteca para comunicación con el dispositivo ELM327 (OBDII) https://github.com/PowerBroker2/ELMduino

#include <WiFi.h>
#include <PubSubClient.h>

//#include <SPIFFS.h>  // Incluye la biblioteca SPIFFS

// Declaramos un objeto BluetoothSerial para manejar la comunicación Bluetooth
BluetoothSerial SerialBT;  

// Definimos el puerto ELM327 para utilizar la comunicación Bluetooth Serial
#define ELM_PORT   SerialBT
#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
// Definimos el puerto de depuración para utilizar la comunicación serial por USB
#define DEBUG_PORT Serial
#else
#define LED_BUILTIN 2  // Define el pin del LED integrado
#endif

// Configuración de Wi-Fi
const char* ssid = "xxxxxxx";
const char* password = "xxxxxxx";

// Configuración de MQTT
const char* mqtt_server = "192.168.xxx.xxx";
const char* mqtt_user = "xxxxxxxxx";
const char* mqtt_password = "xxxxxx";
String mqtt_topic = "OBD2/";

const char *pin = "1234"; // PIN de seguridad Bluetooth
uint8_t address[6] = {0x--, 0x--, 0x--, 0x--, 0x--, 0x--};

WiFiClient espClient;
PubSubClient client(espClient);

// Declaramos un objeto ELM327 para manejar la comunicación con el OBDII
ELM327 myELM327;   

float tempVolts = 0.0;
float tempVoltsOld = 0.0;
float tempRPM = 0.0;
uint32_t Status = 0;
uint16_t codeDTC = 0;

void setup()
{
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(115200);  // Iniciamos el puerto de depuración a una velocidad de 115200 baudios
#endif  
#if LED_BUILTIN
  // Si LED_BUILTIN está definido
  pinMode(LED_BUILTIN, OUTPUT);       // Configuramos el LED como salida
  digitalWrite(LED_BUILTIN, HIGH);    // Enciende el LED
  delay(250);                         // Espera 250ms
  digitalWrite(LED_BUILTIN, LOW);     // Apaga el LED
  delay(1000);                        // Espera 1000ms
  digitalWrite(LED_BUILTIN, HIGH);    // Enciende el LED nuevamente
#else
  // Si LED_BUILTIN no está definido, esperar 1sg.
  delay(1000);
#endif
#ifdef DEBUG_PORT
  DEBUG_PORT.println("DEBUG-->Inicio del setup.");  
#endif
  // Montar SPIFFS y formatear si es necesario
  //if (!SPIFFS.begin(true)) {  // true habilita el formateo si el montaje falla
  //  DEBUG_PORT.println("Error al montar SPIFFS. Formateando...");
  //} else {
  //  DEBUG_PORT.println("SPIFFS montado correctamente");
  //}
 
  delay(100);
  // Iniciamos el Bluetooth con el nombre "Esp32HUD"
  // (true) indica que el ESP32 actuará como servidor
  ELM_PORT.begin("Esp32HUD", true); 
#ifdef DEBUG_PORT
  if (ELM_PORT.setPin(pin, strlen(pin))) {
    DEBUG_PORT.println("DEBUG-->PIN establecido correctamente");
  } else {
    DEBUG_PORT.println("DEBUG-->Error al establecer el PIN");
  }
#endif
  bool connected = ELM_PORT.connect(address);
#ifdef DEBUG_PORT
  DEBUG_PORT.println("DEBUG-->Conectado a dispositivo esclavo BT con MAC");
#endif

  if (!connected)
  { 
#ifdef DEBUG_PORT
    DEBUG_PORT.println("DEBUG-->Fallo la conexión con MAC al esclavo BT");
#endif
    if (!ELM_PORT.connect("OBD BLE"))
    {
#ifdef DEBUG_PORT
      DEBUG_PORT.println("DEBUG-->Fallo la conexión con NOMBRE  al esclavo BT");
 #endif     
#if LED_BUILTIN
      digitalWrite(LED_BUILTIN, HIGH);
#endif
      // Intentamos conectar con el dispositivo OBDII por Bluetooth
      do {    
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->No se pudo conectar al OBD2 - Fase 1");
#endif
        delay (500);
      } while(!ELM_PORT.connect(address)); // Si no se puede conectar, nos quedamos en un bucle infinito
    }
  }
  // Intentamos iniciar la comunicación con el ELM327
  // el segundo parámetro habilita el modo sin bloqueo (non-blocking)
  // el tercer parámetro es el tiempo de espera de la conexión (2000 ms)
  if (!myELM327.begin(ELM_PORT, true, 2000))
  {
    do {
#ifdef DEBUG_PORT
      DEBUG_PORT.println("DEBUG-->No se pudo conectar al ELM327 - Fase 2");
#endif
      delay (1000);
#if LED_BUILTIN
      digitalWrite(LED_BUILTIN, HIGH);
#endif
    } while (!myELM327.begin(ELM_PORT, true, 2000)); // Si no se puede conectar al escáner OBDII, re-intentamos
  }
#ifdef DEBUG_PORT
  // Si la conexión es exitosa
  DEBUG_PORT.println("DEBUG-->Conectado por Bluetooth a ELM327");
  
  DEBUG_PORT.print("DEBUG-->Conectando al AP: ");
  DEBUG_PORT.println(ssid);
  DEBUG_PORT.print("DEBUG-->Intentando conexión WiFi.");
#endif
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
#ifdef DEBUG_PORT
    DEBUG_PORT.print(".");
#endif
    delay(1000);
  }
#ifdef DEBUG_PORT
  DEBUG_PORT.println("");
  DEBUG_PORT.print("DEBUG-->Conectado a WiFi, IP:");
  DEBUG_PORT.println(WiFi.localIP());
#endif
  // Configurar servidor MQTT
  client.setServer(mqtt_server, 1883);
  reconnect();

#if LED_BUILTIN
  digitalWrite(LED_BUILTIN, LOW);     // Apagamos el LED
#endif
}
//***************************MAIN************************************************
unsigned long lastRead = 0;
uint8_t rxData = 0;
void loop()
{
  // Reconectar si la conexión al servidor MQTT se pierde
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

  switch (rxData)
  {
    case 0:
    {
      tempVolts = myELM327.batteryVoltage();
      // Si la lectura de la tensión de batería finalizó correctamente
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.print("DEBUG-->Tension bateria: ");
        DEBUG_PORT.println(tempVolts);
#endif
        rxData = 1;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) // SI NO ES ELM_SUCCESS ni ELM_GETTING_MSG, error de lectuta
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Error al leer el valor de la tension bateria");
#endif
        myELM327.printError();
        rxData = 1;
#if LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
#endif
      }
      break;
    }
    // Obtenemos el estado de los monitores de diagnóstico OBD
    case 1:
    {
      Status = myELM327.monitorStatus();
      // Si la lectura de los monitores de diagnóstico correctamente
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.print("DEBUG-->Estado: ");
        DEBUG_PORT.println(Status, BIN);
#endif
        rxData = 2;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) // SI NO ES ELM_SUCCESS ni ELM_GETTING_MSG, error de lectuta
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Error al leer el valor del monitor de estado");
#endif
        myELM327.printError();
#if LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
#endif
        rxData = 2;
      }
      break;
    }
    case 2:
    {
      // Obtenemos el valor actual del codigo DTC
      codeDTC = myELM327.freezeDTC();
      // Si la lectura del codigo DTC finalizó correctamente
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.print("DEBUG-->Codifgo DTC: ");
        DEBUG_PORT.println(codeDTC);
#endif
        rxData = 3;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) // SI NO ES ELM_SUCCESS ni ELM_GETTING_MSG, error de lectuta
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Error al leer el valor del codigo DTC");
#endif
        myELM327.printError();
#if LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
#endif
        rxData = 3;
      }
      break;
    }
    case 3:
    {
      // Obtenemos el valor actual de las RPM del motor
      tempRPM = myELM327.rpm();
      // Si la lectura de la RPM finalizó correctamente
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.print("DEBUG-->RPM: ");
        DEBUG_PORT.println(tempRPM);
#endif
        rxData = 0;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG) // SI NO ES ELM_SUCCESS ni ELM_GETTING_MSG, error de lectuta
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Error al leer el valor de RPM");
#endif
        myELM327.printError();
#if LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
#endif
        rxData = 0;
      }
      break;
    }
  }
  if ((millis() - lastRead) >= 5000)  // Cada 5 segundos
  {
    lastRead = millis();
#if LED_BUILTIN
      if (digitalRead(LED_BUILTIN))
      {
        digitalWrite(LED_BUILTIN, LOW);
      }
#endif   
    if (tempVolts != tempVoltsOld && tempVolts > 0.0)
    {
      tempVoltsOld = tempVolts;
#ifdef DEBUG_PORT
      DEBUG_PORT.print("DEBUG-->Tension bateria: ");
      DEBUG_PORT.println(tempVolts);
#endif
      String voltsData = String(tempVolts, 2); // Construye el mensaje para MQTT con 2 decinales
      // Enviar los datos de tensión batería al broker MQTT
      if (client.publish((mqtt_topic + "bv").c_str(), voltsData.c_str())) 
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Valor de tension bateria enviado por MQTT");
#endif
      } 
      else 
      {
#ifdef DEBUG_PORT
        DEBUG_PORT.println("DEBUG-->Error al enviar el valor de tension bateria por MQTT");
#endif
      }
    }
    else if (tempVolts > 0.0)
    {
#ifdef DEBUG_PORT
      DEBUG_PORT.println("DEBUG-->Valor de tension bateria NO cambio!!");
#endif
    }
  }
}

void reconnect() {
  while (!client.connected()) {
#ifdef DEBUG_PORT
    DEBUG_PORT.print("DEBUG-->Conectando al servidor MQTT...");
#endif
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
#ifdef DEBUG_PORT
      DEBUG_PORT.println("DEBUG-->OK conectado al servidor MQTT");
#endif
    } else {
#ifdef DEBUG_PORT
      DEBUG_PORT.print("DEBUG-->Falló la conexion MQTT con error: ");
      DEBUG_PORT.print(client.state());
      DEBUG_PORT.println("DEBUG-->Intentando reconectar en 5 segundos...");
#endif
      delay(5000);
    }
  }
}
