#define START_BYTE 0xDD
#define STOP_BYTE  0x77

// Función para calcular el checksum (en este caso, suma de todos los datos)
byte calculateChecksum(byte *data, byte length) {
  byte checksum = 0;
  for(byte i = 0; i < length; i++) {
    checksum += data[i];
  }
  return checksum;
}

void sendFrame(int V, int T, int P, int P1, int P2) {
  byte frame[10];  // Tamaño total del frame: 1 start byte, 1 length byte, 6 data bytes, 1 checksum byte, 1 stop byte
  
  // Armar el frame
  frame[0] = START_BYTE;
  frame[1] = 20;  // Longitud de los datos (3 floats de 4 bytes cada uno)
  
  // Copiar los datos float en el frame
  memcpy(&frame[2], &V, 4);  // Copia de 4 bytes del float V
  memcpy(&frame[6], &T, 4);  // Copia de 4 bytes del float T
  memcpy(&frame[10], &P, 4); // Copia de 4 bytes del float P
  memcpy(&frame[14], &P, 4); // Copia de 4 bytes del float P1
  memcpy(&frame[18], &P, 4); // Copia de 4 bytes del float P2
  
  // Calcular el checksum
  frame[22] = calculateChecksum(&frame[2], 20);
  
  // Añadir el stop byte
  frame[23] = STOP_BYTE;
  
  // Enviar el frame completo
  Serial.write(frame, sizeof(frame));
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  float V = 1023;  // Ejemplo de valor para V
  float T = 500;   // Ejemplo de valor para T
  float P = 250;   // Ejemplo de valor para P

  sendFrame(V, T, P, P1, P2);
  delay(1000);  // Espera 1 segundo antes de enviar el siguiente frame
}
