// Credenciales de Blynk

#define BLYNK_TEMPLATE_ID "TMPLWr3cRC-X"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "VxTXFmCytQB3mVpi80GcTqVF62TND4VR"
#define BLYNK_PRINT Serial

// Librerías utilizadas

#include <Arduino.h> // Permite deinir los GPIOs, su función (entrada o salida), etc. Incluye la librería math.h. https://github.com/esp8266/Arduino/blob/master/cores/esp8266/Arduino.h
#include <MQ135.h> // Para el sensor de CO2. https://github.com/NuclearPhoenixx/MQ135/blob/master/MQ135.cpp
#include <DHT.h> // Para el sensor de temperatura y HR. https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
//#include <math.h> // Para hacer cálculos. 
#include <OneWire.h>
#include <WiFi.h>
#include <WiFiMulti.h> // Para conectarse a varias redes WiFi
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h> // Para el sensor de CE
#include <RTClib.h> // Para l reloj
#include <SD.h> // Para la SD

// pines de los sensores

#define hum_torres_pin 34
#define hum_temp_amb_pin 16
#define CO2_pin 39
#define temperatura_agua_pin 33
#define ph_pin 35
#define ce_pin 32

// pines de los actuadores

#define Ventilador 2
#define Motobomba 4

// Credenciales del WIFI

char ssid1[] = "Familia castillo ";
char pass1[] = "00952882038";

char ssid2[] = "Familia vargas_2";
char pass2[] = "Camila500%";

char ssid3[] = "David Agudelo";
char pass3[] = "Juacoywanda13";

// Estado inicial de los acuadors

bool Ventilador_activado = true; // Ventilador encendido
bool Motobomba_activada = true; // Motobomba encendida

// Variables para almacenar la información de los sensores. Se definen valores iniciales de 0 para no presentar interferencia en los valores iniciales.

float humedad_relativa = 0;
float temperatura_ambiental = 0;
float VPD = 0;
float humedad_raices = 0;
float valor_tds = 0;
float temperatura_agua = 0;
float CE_torres = 0;
float pH_torres = 0;
float milivoltios_pH = 0;
float CO2 = 0;

// Variables para control de actuadores

float t_min = 22;
float t_max = 25;
float VPD_lim = 1.5;

// Variables del temporizaador de riego

unsigned long lastActivationTime = 0; // Variable para rastrear el tiempo de la última activación del relé
const unsigned long activationInterval = 15 * 60 * 1000; // Intervalo de activación en milisegundos (15 minutos)
const unsigned long activationDuration = 2 * 60 * 1000; // Duración de la activación en milisegundos (2 minutos)

// Variables de la lectura de CO2

MQ135 sensor_CO2 = MQ135(CO2_pin); // Se crea un objeto (sensor_CO2) de clase "MQ135", la cual encasula la funcinalidad del sensor conectado al pin "CO2_pin"
float rzero = sensor_CO2.getRZero(); // Se crea una variable que alacena el factor de corrección del sensor de CO2

RTC_DS1307 rtc;
DHT dht = DHT(hum_temp_amb_pin, DHT22);
int hora_inicio = 6;
int hora_fin = 21;

WiFiMulti wifiMulti;
const uint32_t TiempoEsperaWifi = 1000;

// CE

#define VREF 3.3           // Valor de voltaje de referencia, es decir, el que se utiliza por el módulo de pH
#define SCOUNT 30          // Define el número de datos con los cuales se realizará un promedio
int analogBuffer[SCOUNT];  // Almacena los valores leidos, en una matríz de 30 datos
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;

// Temperatura del agua

OneWire oneWire = OneWire(temperatura_agua_pin);
DallasTemperature dallasTemperature = DallasTemperature(&oneWire);

// Blynk

BlynkTimer timer;

// Parámetros iniciales del sistema

void setup() {

  // Inicialización del monitor serial y sensores

  Serial.begin(115200); // Se define la velocidad de comunicación con el computador, en bps (bits por segundo)
  Wire.begin();
  rtc.begin(); // Se inicializa el reloj
  dht.begin(); // Se nicializa el sensor de t y h
  dallasTemperature.begin();

  // Definición de entradas y salidas

  pinMode(hum_temp_amb_pin, INPUT); //Los INPUT son datos que van a ingresar al programa, es decir, estos GPIO serán entradas de datos
  pinMode(hum_torres_pin, INPUT);
  pinMode(CO2_pin, INPUT);
  pinMode(ce_pin, INPUT);
  pinMode(temperatura_agua_pin, INPUT);
  pinMode(ph_pin, INPUT);

  pinMode(Ventilador, OUTPUT); // Los OUTPUT son datos que salen del programa, en este caso el encendido o apagado de un relé que enciende o apaga actuadores
  pinMode(Motobomba, OUTPUT);

  // Definición del estado inicial de los actuadores

  digitalWrite(Ventilador, LOW); // Estado LOW es encendido
  digitalWrite(Motobomba, LOW);

  // Conexión a Wifi

  wifiMulti.addAP(ssid1, pass1);
  wifiMulti.addAP(ssid2, pass2);
  wifiMulti.addAP(ssid3, pass3);

  WiFi.mode(WIFI_STA);
  while(wifiMulti.run(TiempoEsperaWifi) != WL_CONNECTED){
    Serial.print("Conectando...");
    control_de_actuadores();
  }

  String ssid = WiFi.SSID();
  String pass;

  Serial.print("Conectado a ");
  Serial.print(ssid);

  if(ssid == ssid1) {
    pass = pass1;
  } else if(ssid == ssid2) {
    pass = pass2;
  } else {
    pass = pass3;
  }

  //Inicialización de la comunicación con Blynk

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid.c_str(), pass.c_str());
  delay(1000);
}

void datos_sensores() {

  // Lecturas de sensores
  humedad_relativa = dht.readHumidity();
  temperatura_ambiental = dht.readTemperature();
  VPD = calcular_VPD();
  humedad_raices = map(analogRead(hum_torres_pin), 0, 4095, 100, 0);
  CO2 = sensor_CO2.getCorrectedPPM(temperatura_ambiental, humedad_relativa);

  dallasTemperature.requestTemperatures();  // Lectura de temperatura del agua
  temperatura_agua = dallasTemperature.getTempCByIndex(0);

  valor_ce(); // Lectura y procesamiento de sensor de conductividad eléctrica (CE)

  valor_ph(); // Lectura y procesamiento de sensor de pH

  Enviar_datos_blynk(); // Envío de datos a Blynk
}

float valor_ce() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(ce_pin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
    float compensationVolatge = calculateTemperatureCompensation(averageVoltage);
    valor_tds = convertVoltageToTDS(compensationVolatge);

    CE_torres = valor_tds / 640;
  }
}

void valor_ph() {
  float pHsum = 0.0;
  float voltagesum = 0.0;
  float slope = -4.4118;     // pendiente de la ecuación de calibración
  float intercept = 17.973;  // intercepto de la ecuación de calibración

  for (int i = 0; i < 30; i++) {
    int rawValue = analogRead(ph_pin);
    float voltage = rawValue * (3.3 / 4095.0);
    float pH = slope * voltage + intercept;
    pHsum += pH;
    voltagesum += voltage;
    delay(10);
  }

  pH_torres = pHsum / 30.0;
  milivoltios_pH = voltagesum / 30;
}

void Enviar_datos_blynk() {
  // Envío de datos a Blynk
  Blynk.virtualWrite(V1, temperatura_agua);
  Blynk.virtualWrite(V2, valor_tds);
  Blynk.virtualWrite(V3, CE_torres);
  Blynk.virtualWrite(V4, pH_torres);
  Blynk.virtualWrite(V5, humedad_relativa);
  Blynk.virtualWrite(V6, temperatura_ambiental);
  Blynk.virtualWrite(V7, milivoltios_pH);
  Blynk.virtualWrite(V8, VPD);
  Blynk.virtualWrite(V9, humedad_raices);
  Blynk.virtualWrite(V10, CO2);
  Serial.println("Dato enviado a Blynk");
}

float calcular_VPD() {
  // Cálculo de VPD
  return (610.78 / 1000 * (pow(2.71818, ((temperatura_ambiental * 17.2694) / (temperatura_ambiental + 237.3))))) * ((100 - humedad_relativa) / 100);
}

float calculateTemperatureCompensation(float averageVoltage) {
  // Compensación de temperatura

  float temperatura_agua_actual = 0;

  if(dallasTemperature.isConversionComplete()){
    temperatura_agua_actual = temperatura_agua;
  }else{
    temperatura_agua_actual = 18;;
  }

  float compensationCoefficient = 1.0 + 0.02 * (temperatura_agua_actual - 25.0); // Acá se hace una corrección por la temperatura del agua, pero como el sensor no sirve, se cambió
  return averageVoltage / compensationCoefficient;
}

float convertVoltageToTDS(float compensationVolatge) {
  // Conversión de valor de voltaje a valor de TDS
  return (133.42 * pow(compensationVolatge, 3) - 255.86 * pow(compensationVolatge, 2) + 857.39 * compensationVolatge) * 0.5;
}


float getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void control_de_actuadores() {

  DateTime now = rtc.now();  // Obtiene la fecha y hora actual del RTC

  if (now.hour() >= hora_inicio && now.hour() < hora_fin) { // Verifica si es hora de encender los actuadores (entre las 6 am y las 7 pm)
    unsigned long currentTime = millis();
    if (currentTime - lastActivationTime >= activationInterval) { // Verifica si ha pasado suficiente tiempo desde la última activación
      digitalWrite(Motobomba, LOW); // Activa el relé
      lastActivationTime = currentTime; // Actualiza el tiempo de la última activación
      Motobomba_activada = true; // Marca el relé como activado
    }
    
    if (Motobomba_activada && currentTime - lastActivationTime >= activationDuration) { // Verifica si el relé ha estado activado durante 3 minutos
      digitalWrite(Motobomba, HIGH); // Desactiva el relé después de 3 minutos
      Motobomba_activada = false; // Marca el relé como desactivado
    }

    if (temperatura_ambiental > t_max) { //Verifica s la temperatura está por encima de la temperatura máxima establecida
      digitalWrite(Ventilador, LOW); // Activa el relé
      Ventilador_activado = true; //Marca el relé como activado
    } else if (temperatura_ambiental < t_min) { //Verifica si la temperatura está por debajo de la temperatura mínima establecida
      digitalWrite(Ventilador, HIGH); // Desactiva el relé
      Ventilador_activado = false; // Marca el relé como desactivado
    } else { // Esto permite que se encienda al alcanzar el máxmo, pero se apague nuevamente hasta que esté por debajo del mínimo, es decir, mantiene el estado actual
    }

  } else { // Verifica si es hora de apagar los actuadores
    // Fuera del rango de tiempo, apaga
    digitalWrite(Ventilador, HIGH); // Desactiva el ventilador
    digitalWrite(Motobomba, HIGH); // Desactiva la motobomba
  }
}

void loop() {
  DateTime now = rtc.now();

  Serial.printf("%02d/%02d/%04d %02d:%02d:%02d\n", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());

  control_de_actuadores();
  
  if (Blynk.connected()) {
    Blynk.run();
    timer.run();
  } else {
    Serial.println("Reconectando a Blynk...");
    Blynk.connect();
    delay(1000);  // Puedes ajustar este tiempo según sea necesario
  }
}