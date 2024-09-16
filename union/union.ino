#define START_BYTE 0xDD
#define STOP_BYTE  0x77
#define scaleFactor 10 // Ajusta este valor según la escala de tu torqueímetro

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27,16,2);  //


const int analogPin = A0;  // Pin analógico al que está conectada la salida del amplificador diferencial
float voltaget = 0.0;       // Variable para almacenar el voltaje leído
float torque = 0.0;        // Variable para almacenar el torque calculado
const float V_REF = 5.0;   // Voltaje de referencia del Arduino (5V)
const int ADC_RES = 1023;  // Resolución del ADC del Arduino (1023 para 10 bits)

// Definir el pin de entrada digital en el Arduino Mega
const int pinSensor = 2;

// Variables para el conteo de pulsos y el tiempo
volatile unsigned long contadorPulsos = 0;
unsigned long tiempoAnterior = 0;

// Velocidad angular en radianes por segundo
float velocidadAngular = 0.0;

// Constantes del sistema
const int incrementosPorVuelta = 2000;                                 // Número de incrementos por vuelta del codificador
const float gradosPorIncremento = 360.0 / (incrementosPorVuelta * 4);  // Grados por incremento (4X decodificación)
const unsigned long intervaloTiempo = 100;                             // Intervalo de tiempo en ms para la velocidad angular
float velocidadRevMin = 0;

// Definir los pines analógicos donde están conectados los sensores
const int SensorPresion1 = A1;  // Pin analógico para el primer sensor
const int SensorPresion2 = A2;  // Pin analógico para el segundo sensor

// Definir las constantes del sensor
const float voltajeMin = 0.5;   // Voltaje mínimo del sensor (en V)
const float voltajeMax = 4.5;   // Voltaje máximo del sensor (en V)
const float presionMin = 0.0;   // Presión mínima (en psi)
const float presionMax = 200.0;  // Presión máxima (en psi)

// Variables para almacenar los valores leídos
float voltajepresion1, voltajepresion2;
float presion1, presion2, diferencialPresion;

// Función para calcular el checksum (en este caso, suma de todos los datos)
byte calculateChecksum(byte* data, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];  // XOR simple para checksum
  }
  return checksum;
}

void setup() {
  // Iniciar la comunicación serie
  Serial.begin(9600);
  
  // Inicializar el LCD
  lcd.init();
  
  //Encender la luz de fondo.
  lcd.backlight();

  // Configurar el pin de entrada digital
  pinMode(pinSensor, INPUT);

  // Configurar la interrupción para contar los pulsos en el pin 2
  attachInterrupt(digitalPinToInterrupt(pinSensor), contarPulsos, RISING);
}

void loop() {
  velocidadAngular = Vangular();
  torque = Ftorque();
  diferencialPresion = presion();
  pantalla(velocidadAngular, torque, diferencialPresion);
  // Imprimir los valores para verificar que son correctos antes de enviarlos
/*   Serial.println("");
  Serial.print("Velocidad Angular: ");
  Serial.println(velocidadAngular);
  Serial.print("Torque: ");
  Serial.println(torque);
  Serial.print("Diferencial de Presión: ");
  Serial.println(diferencialPresion); */
  
  sendFrame(velocidadAngular, torque, diferencialPresion, presion1, presion2);
  delay(1000);  // Espera 1 segundos antes de enviar el siguiente frame
}

void printWithSpaces(float value, int width, int decimals = 0) {
  String valueStr;

  // Si se especifican decimales, redondear y formatear el valor como flotante
  if (decimals > 0) {
    valueStr = String(value, decimals);
  } else {
    // Redondear al entero más cercano
    valueStr = String((int)value);
  }

  // Añadir espacios a la izquierda si es necesario
  while (valueStr.length() < width) {
    valueStr = " " + valueStr;
  }

  // Imprimir el valor con el LCD
  lcd.print(valueStr);
}

void pantalla(float velocidadAngular, float torque, float diferencialPresion){
  lcd.setCursor(0, 0);
  lcd.print("V");
  printWithSpaces(velocidadAngular, 5);  // Mostrar con 3 caracteres
  lcd.print("rev/min");
  
  // Cursor en la 11ª posición de la primera fila
  lcd.setCursor(9, 1);
  lcd.print("T");
  printWithSpaces(torque, 4,1);  // Mostrar con 3 caracteres
  lcd.print("Nm");
  
  // Cursor en la primera posición de la segunda fila
  lcd.setCursor(0, 1);
  lcd.print("P");
  printWithSpaces(diferencialPresion, 4);  // Mostrar con 3 caracteres
  lcd.print("psi");
}

void sendFrame(float V, float T, float P, float P1, float P2) {
  byte frame[24];  // Tamaño total del frame: 1 start byte, 1 length byte, 20 data bytes, 1 checksum byte, 1 stop byte
  
  // Armar el frame
  frame[0] = START_BYTE;
  frame[1] = 20;  // Longitud de los datos (5 floats de 4 bytes cada uno)
  
  // Copiar los datos float en el frame
  memcpy(&frame[2], &V, 4);  // Copia de 4 bytes del float V
  memcpy(&frame[6], &T, 4);  // Copia de 4 bytes del float T
  memcpy(&frame[10], &P, 4); // Copia de 4 bytes del float P
  memcpy(&frame[14], &P1, 4); // Copia de 4 bytes del float P1
  memcpy(&frame[18], &P2, 4); // Copia de 4 bytes del float P2
  
  // Calcular el checksum
  frame[22] = calculateChecksum(&frame[2], 20);
  
  // Añadir el stop byte
  frame[23] = STOP_BYTE;
  
  // Enviar el frame completo
  Serial.write(frame, sizeof(frame));
}

void printFrameRange(byte* frame, int start, int end) {
  for (int i = start; i <= end; i++) {
    Serial.print("0x");
    if (frame[i] < 0x10) {
      Serial.print("0");  // Para asegurar que siempre tenga dos dígitos
    }
    Serial.print(frame[i], HEX);
    Serial.print(" ");  // Espacio para separar los bytes en la salida
  }
  Serial.println();  // Nueva línea después de imprimir el rango
}

float Vangular() {
  // Calcular el tiempo transcurrido
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= intervaloTiempo) {
    // Calcular la velocidad angular en grados por segundo
    float velocidadGradosPorSegundo = (contadorPulsos * gradosPorIncremento) / (intervaloTiempo / 1000.0);

    // Convertir la velocidad angular a revoluciones por minuto
    velocidadRevMin = velocidadGradosPorSegundo / 6.0;  // Dividir por 6 para convertir a rev/min

    // Reiniciar el contador y el tiempo
    contadorPulsos = 0;
    tiempoAnterior = tiempoActual;
  }
  
  return velocidadRevMin;
}

float Ftorque() {
  int sensorValue = analogRead(analogPin);    // Lee el valor analógico
  voltaget = (sensorValue * V_REF) / ADC_RES;  // Convierte el valor analógico a voltaje

  // Calcula el torque basado en el voltaje medido
  torque = (voltaget - 2.5) * scaleFactor;  // Ajusta el factor de escala
  /*
  Serial.print("Voltaje: ");
  Serial.print(voltaget);
  Serial.print(" V, Torque: ");
  Serial.print(torque);
  Serial.println(" Nm");  // Ajusta la unidad según la salida de tu torqueímetro
  */
  return torque;
}

float presion() {
  // Leer los valores analógicos de los sensores
  int lecturaAnalogica1 = analogRead(SensorPresion1);
  int lecturaAnalogica2 = analogRead(SensorPresion2);

  // Convertir las lecturas analógicas (0-1023) a voltaje (0-5V)
  voltajepresion1 = lecturaAnalogica1 * (4.5 / 1023.0);
  // Convertir voltaje (0.5V a 4.5V) a presión (0 psi a 200 psi)
  voltajepresion1= ((voltajepresion1 - 0.5) / (4.5 - 0.5)) * 200;

  voltajepresion2 = lecturaAnalogica2 * (4.5 / 1023.0);
  voltajepresion2= ((voltajepresion2 - 0.5) / (4.5 - 0.5)) * 200;

  // Calcular la presión para cada sensor basada en el voltaje leído
  presion1 = mapearVoltajeAPresion(voltajepresion1);
  presion2 = mapearVoltajeAPresion(voltajepresion2);

  // Calcular la diferencia de presión
  diferencialPresion = presion1 - presion2;
/*
  // Imprimir los resultados en la consola serie
  Serial.print("Sensor 1 - Voltaje: ");
  Serial.print(voltajepresion1);
  Serial.print(" V | Presión: ");
  Serial.print(presion1);
  Serial.print(" psi");

  Serial.print(" | Sensor 2 - Voltaje: ");
  Serial.print(voltajepresion2);
  Serial.print(" V | Presión: ");
  Serial.print(presion2);
  Serial.print(" psi");

  Serial.print(" | Diferencia de Presión: ");
  Serial.print(diferencialPresion);
  Serial.println(" psi");
*/
  return presion1,presion2,diferencialPresion;
}

// Función para mapear el voltaje a la presión
float mapearVoltajeAPresion(float voltaje) {
  if (voltaje < voltajeMin) {
    return presionMin;
  } else if (voltaje > voltajeMax) {
    return presionMax;
  } else {
    return (voltaje - voltajeMin) * (presionMax - presionMin) / (voltajeMax - voltajeMin) + presionMin;
  }
}

void contarPulsos() {
  // Incrementar el contador de pulsos en cada flanco ascendente
  contadorPulsos++;
}
