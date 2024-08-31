// Definir los pines analógicos donde están conectados los sensores
const int SensorPresion1 = A1; // Pin analógico para el primer sensor
const int SensorPresion2 = A2; // Pin analógico para el segundo sensor

// Definir las constantes del sensor
const float voltajeMin = 0.5; // Voltaje mínimo del sensor (en V)
const float voltajeMax = 4.5; // Voltaje máximo del sensor (en V)
const float presionMin = 0.0; // Presión mínima (en MPa)
const float presionMax = 20.0; // Presión máxima (en MPa)

// Variables para almacenar los valores leídos
float voltajeSensor1, voltajeSensor2;
float presion1, presion2, diferencialPresion;

// Función de configuración
void setup() {
  // Iniciar la comunicación serie
  Serial.begin(9600);
}

// Función principal
void loop() {
  // Leer los valores analógicos de los sensores
  int lecturaAnalogica1 = analogRead(SensorPresion1);
  int lecturaAnalogica2 = analogRead(SensorPresion2);

  // Convertir las lecturas analógicas (0-1023) a voltaje (0-5V)
  voltajeSensor1 = lecturaAnalogica1 * (5.0 / 1023.0);
  voltajeSensor2 = lecturaAnalogica2 * (5.0 / 1023.0);

  // Calcular la presión para cada sensor basada en el voltaje leído
  presion1 = mapearVoltajeAPresion(voltajeSensor1);
  presion2 = mapearVoltajeAPresion(voltajeSensor2);

  // Calcular la diferencia de presión
  diferencialPresion = presion1 - presion2;

  // Imprimir los resultados en la consola serie
  Serial.print("Sensor 1 - Voltaje: ");
  Serial.print(voltajeSensor1);
  Serial.print(" V | Presión: ");
  Serial.print(presion1);
  Serial.print(" MPa");

  Serial.print(" | Sensor 2 - Voltaje: ");
  Serial.print(voltajeSensor2);
  Serial.print(" V | Presión: ");
  Serial.print(presion2);
  Serial.print(" MPa");

  Serial.print(" | Diferencia de Presión: ");
  Serial.print(diferencialPresion);
  Serial.println(" MPa");

  delay(1000); // Pausar 1 segundo entre lecturas
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
