const int analogPin = A0; // Pin analógico al que está conectada la salida del amplificador diferencial
float voltage = 0.0;      // Variable para almacenar el voltaje leído
float torque = 0.0;       // Variable para almacenar el torque calculado
const float V_REF = 5.0;  // Voltaje de referencia del Arduino (5V)
const int ADC_RES = 1023; // Resolución del ADC del Arduino (1023 para 10 bits)

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(analogPin); // Lee el valor analógico
  voltage = (sensorValue * V_REF) / ADC_RES; // Convierte el valor analógico a voltaje

  // Calcula el torque basado en el voltaje medido
  // Suponiendo que la relación es lineal: 2.5V es 0 torque
  torque = (voltage - 2.5) * scaleFactor(); // Ajusta el factor de escala

  Serial.print("Voltaje: ");
  Serial.print(voltage);
  Serial.print(" V, Torque: ");
  Serial.print(torque);
  Serial.println(" Nm"); // Ajusta la unidad según la salida de tu torqueímetro

  delay(500); // Espera medio segundo antes de la siguiente lectura
}

float scaleFactor() {
  return 10.0; // Ajusta este valor según la escala de tu torqueímetro
}

