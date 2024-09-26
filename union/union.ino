// Libraries includes
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

//Interval definitions in seconds
#define MeasureInterval 10000
#define intervaloTiempo 100 // Time interval (ms) for speed calculation

// Messages definitions
#define START_BYTE 0xDD
#define STOP_BYTE  0x77
#define scaleFactor 10 // Adjust this according to your torque sensor's scale
#define STAR_MESSAGE 0x4D
#define STOP_MESSAGE 0x5A

// Sensor related definitions
#define analogPin 0     // Analog pin for differential amplifier output
#define pinSensor 2     // Digital input pin for pulse counting
#define SensorPresion1 1
#define SensorPresion2 2

// Constant definitions
#define V_REF 5.0         // Reference voltage for Arduino (5V)
#define ADC_RES 1023      // ADC resolution (1023 for 10-bit)
#define voltajeMin 0.5    // Min sensor voltage (V)
#define voltajeMax 4.5    // Max sensor voltage (V)
#define presionMin 0.0    // Min pressure (psi)
#define presionMax 200.0  // Max pressure (psi)
#define incrementosPorVuelta 2000 // Increments per encoder turn
#define gradosPorIncremento (360.0 / (incrementosPorVuelta * 4)) // Degrees per increment

// Working variables
volatile unsigned long contadorPulsos = 0; // Pulse count variable
unsigned long tiempoAnterior = 0; // Track time
float velocidadRevMin = 0;
int countMeas = 0;

// Flags
bool star_flag = false;
bool testsFailed = false;
byte line = 0;

//Interruption related variables
bool timer_one_flag = true;

//Sensor related variables
typedef struct sensorDataType {
    float torqueAcum;
    float torque;           // Store calculated torque
    float velocidadAngularAcum;
    float velocidadAngular; // Angular speed in radians/second
};

sensorDataType dataRead = {0};

// Message variable variables
typedef struct presionTipo {
  float presion1Acum;
  float presion1;
  float presion2Acum;
  float presion2;
  float deltapAcum;
  float deltap;
};

presionTipo datos_presion = {0};


//----------------------------------------------------------------------------------------------


// Initialize LCD object (address 0x27, 16 columns x 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Checksum function (XOR)
byte calculateChecksum(byte* data, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void setup() {
  // Serial related setup
  Serial.begin(9600);  // Start serial communication

  // LCD related setup
  lcd.init();          // Initialize LCD
  lcd.backlight();     // Turn on backlight

  // Pin related setup
  pinMode(pinSensor, INPUT);  // Set pin as input

  // Interruption realated setup
  attachInterrupt(digitalPinToInterrupt(pinSensor), contarPulsos, RISING);  // Setup interrupt

  // Timer 1 (1 sec)
  Timer1.initialize(MeasureInterval); // 10000 microsegundos = 0.01 segundo
  Timer1.attachInterrupt(timerCallback1); 

}

void loop() {
  if (Serial.available() > 0){
    line = Serial.read();
    if (line == STAR_MESSAGE){
      star_flag = true;
    }
    else if (line == STOP_MESSAGE){
      star_flag = false;
    }
  }

  if (timer_one_flag){
      Acumulation();
      presionAcum();

      countMeas ++;
      timer_one_flag = false;
  }

  if (countMeas == 100){
      UpdateData();
    
      pantalla(dataRead.velocidadAngular, dataRead.torque, datos_presion.deltap);  // Update LCD display
      
      if (star_flag){
        sendFrame(dataRead.velocidadAngular, dataRead.torque, datos_presion.deltap, datos_presion.presion1, datos_presion.presion2);
      }
    
      countMeas = 0;
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

// Function to print data on LCD
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
  float voltaget = (sensorValue * V_REF) / ADC_RES;
  float torque;

  torque = (voltaget - 2.5) * scaleFactor;
  return torque;
}

// Angular velocity and Torque Acumulation
void Acumulation() {
  int sensorValue = analogRead(analogPin);
  float voltaget = (sensorValue * V_REF) / ADC_RES;
  float velocidadGradosPorSegundo = (contadorPulsos * gradosPorIncremento) / (intervaloTiempo / 1000.0);

  velocidadRevMin = velocidadGradosPorSegundo / 6.0;
  contadorPulsos = 0;

  dataRead.velocidadAngularAcum += velocidadRevMin;
  dataRead.torqueAcum += (voltaget - 2.5) * scaleFactor;
  
  return;
}

// Pressure data Acumulation
void presionAcum(){
  int lecturaAnalogica1 = analogRead(SensorPresion1);
  int lecturaAnalogica2 = analogRead(SensorPresion2);
  float voltajepresion1 = lecturaAnalogica1 * (V_REF / 1023.0);
  float voltajepresion2 = lecturaAnalogica2 * (V_REF / 1023.0);
  
  voltajepresion1 = ((voltajepresion1 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  voltajepresion2 = ((voltajepresion2 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  datos_presion.presion1Acum += abs(voltajepresion1);
  datos_presion.presion2Acum += abs(voltajepresion2);
  datos_presion.deltapAcum += datos_presion.presion1 - datos_presion.presion2;

  return;
}

// Update the sended data
void UpdateData(){
  datos_presion.presion1 = datos_presion.presion1Acum / countMeas;
  datos_presion.presion2 = datos_presion.presion2Acum/ countMeas;
  datos_presion.deltap = datos_presion.deltapAcum / countMeas;
  dataRead.velocidadAngular = dataRead.velocidadAngularAcum / countMeas;
  dataRead.torque = dataRead.torqueAcum / countMeas;

  datos_presion.presion1Acum = 0;
  datos_presion.presion2Acum = 0;
  datos_presion.deltapAcum = 0;
  dataRead.velocidadAngularAcum = 0;
  dataRead.torqueAcum = 0;
}

// Pressure calculation
presionTipo presion() {
  int lecturaAnalogica1 = analogRead(SensorPresion1);
  int lecturaAnalogica2 = analogRead(SensorPresion2);
  presionTipo datos_presion_local = {0};

  float voltajepresion1 = lecturaAnalogica1 * (V_REF / 1023.0);
  voltajepresion1 = ((voltajepresion1 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  float voltajepresion2 = lecturaAnalogica2 * (V_REF / 1023.0);
  voltajepresion2 = ((voltajepresion2 - voltajeMin) / (voltajeMax - voltajeMin)) * presionMax;

  datos_presion_local.presion1 = abs(voltajepresion1);
  datos_presion_local.presion2 = abs(voltajepresion2);
  datos_presion_local.deltap = datos_presion_local.presion1 - datos_presion_local.presion2;

  return datos_presion_local;
}

// Count pulses
void contarPulsos() {
  contadorPulsos++;
}

void timerCallback1() {
  timer_one_flag = true;
}
