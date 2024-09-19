#define START_BYTE 0xDD
#define STOP_BYTE  0x77
#define scaleFactor 10 // Adjust this according to your torque sensor's scale
#define STAR_MESSAGE 0x4D
#define STOP_MESSAGE 0x5A

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD object (address 0x27, 16 columns x 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int analogPin = A0;  // Analog pin for differential amplifier output
float voltaget = 0.0;       // Store the read voltage
float torque = 0.0;        // Store calculated torque
const float V_REF = 5.0;   // Reference voltage for Arduino (5V)
const int ADC_RES = 1023;  // ADC resolution (1023 for 10-bit)

const int pinSensor = 2;   // Digital input pin for pulse counting

volatile unsigned long contadorPulsos = 0; // Pulse count variable
unsigned long tiempoAnterior = 0; // Track time
float velocidadAngular = 0.0; // Angular speed in radians/second

const int incrementosPorVuelta = 2000; // Increments per encoder turn
const float gradosPorIncremento = 360.0 / (incrementosPorVuelta * 4); // Degrees per increment
const unsigned long intervaloTiempo = 100; // Time interval (ms) for speed calculation
float velocidadRevMin = 0;

// Analog pins for pressure sensors
const int SensorPresion1 = A1;
const int SensorPresion2 = A2;

const float voltajeMin = 0.5;   // Min sensor voltage (V)
const float voltajeMax = 4.5;   // Max sensor voltage (V)
const float presionMin = 0.0;   // Min pressure (psi)
const float presionMax = 200.0; // Max pressure (psi)

//flags
bool star_flag = false;

byte line = 0;

typedef struct presionTipo {
  float presion1;
  float presion2;
  float deltap;
} presionTipo;

presionTipo datos_presion = {0};

// Checksum function (XOR)
byte calculateChecksum(byte* data, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void setup() {
  Serial.begin(9600);  // Start serial communication
  lcd.init();          // Initialize LCD
  lcd.backlight();     // Turn on backlight

  pinMode(pinSensor, INPUT);  // Set pin as input
  attachInterrupt(digitalPinToInterrupt(pinSensor), contarPulsos, RISING);  // Setup interrupt
}

void loop() {
  if (Serial.available() > 0){
    line = Serial.read();
  
    if (line == STAR_MESSAGE){
      star_flag = true;
    }

    if (line == STOP_MESSAGE){
      star_flag = false;
    }

    if (star_flag){
      velocidadAngular = Vangular();  // Calculate angular velocity
      torque = Ftorque();             // Calculate torque
      datos_presion = presion();       // Get pressure readings
    
      pantalla(velocidadAngular, torque, datos_presion.deltap);  // Update LCD display

      sendFrame(velocidadAngular, torque, datos_presion.deltap, datos_presion.presion1, datos_presion.presion2);
    
      delay(1000);  // Delay 1 second before sending the next frame
    }
  }
}

// Function to print formatted data on LCD
void printWithSpaces(float value, int width, int decimals = 0) {
  String valueStr = (decimals > 0) ? String(value, decimals) : String((int)value);
  
  while (valueStr.length() < width) {
    valueStr = " " + valueStr;
  }
  
  lcd.print(valueStr);
}

void pantalla(float velocidadAngular, float torque, float diferencialPresion) {
  lcd.setCursor(0, 0);
  lcd.print("V:");
  printWithSpaces(velocidadAngular, 5);
  lcd.print("rpm");

  lcd.setCursor(9, 1);
  lcd.print("T:");
  printWithSpaces(torque, 4, 1);
  lcd.print("Nm");

  lcd.setCursor(0, 1);
  lcd.print("P:");
  printWithSpaces(diferencialPresion, 4);
  lcd.print("psi");
}

// Function to send data frame over Serial
void sendFrame(float V, float T, float P, float P1, float P2) {
  byte frame[24];
  frame[0] = START_BYTE;
  frame[1] = 20;
  
  memcpy(&frame[2], &V, 4);
  memcpy(&frame[6], &T, 4);
  memcpy(&frame[10], &P, 4);
  memcpy(&frame[14], &P1, 4);
  memcpy(&frame[18], &P2, 4);

  frame[22] = calculateChecksum(&frame[2], 20);
  frame[23] = STOP_BYTE;

  Serial.write(frame, sizeof(frame));
}

// Angular velocity calculation
float Vangular() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= intervaloTiempo) {
    float velocidadGradosPorSegundo = (contadorPulsos * gradosPorIncremento) / (intervaloTiempo / 1000.0);
    velocidadRevMin = velocidadGradosPorSegundo / 6.0;

    contadorPulsos = 0;
    tiempoAnterior = tiempoActual;
  }
  return velocidadRevMin;
}

// Torque calculation
float Ftorque() {
  int sensorValue = analogRead(analogPin);
  voltaget = (sensorValue * V_REF) / ADC_RES;

  torque = (voltaget - 2.5) * scaleFactor;
  return torque;
}

// Pressure calculation
presionTipo presion() {
  int lecturaAnalogica1 = analogRead(SensorPresion1);
  int lecturaAnalogica2 = analogRead(SensorPresion2);

  presionTipo datos_presion = {0};

  float voltajepresion1 = lecturaAnalogica1 * (V_REF / 1023.0);
  voltajepresion1 = ((voltajepresion1 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  float voltajepresion2 = lecturaAnalogica2 * (V_REF / 1023.0);
  voltajepresion2 = ((voltajepresion2 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  datos_presion.presion1 = abs(voltajepresion1);
  datos_presion.presion2 = abs(voltajepresion2);
  datos_presion.deltap = datos_presion.presion1 - datos_presion.presion2;

  return datos_presion;
}

// Count pulses
void contarPulsos() {
  contadorPulsos++;
}