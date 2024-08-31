 // Definir el pin de entrada digital en el Arduino Mega
const int pinSensor = 2;

// Variables para el conteo de pulsos y el tiempo
volatile unsigned long contadorPulsos = 0;
unsigned long tiempoAnterior = 0;

// Velocidad angular en radianes por segundo
float velocidadAngular = 0.0;

// Constantes del sistema
const int incrementosPorVuelta = 2000; // Número de incrementos por vuelta del codificador
const float gradosPorIncremento = 360.0 / (incrementosPorVuelta * 4); // Grados por incremento (4X decodificación)
const unsigned long intervaloTiempo = 100; // Intervalo de tiempo en ms para la velocidad angular

void setup() {
  // Iniciar la comunicación serie
  Serial.begin(9600);

  // Configurar el pin de entrada digital
  pinMode(pinSensor, INPUT);

  // Configurar la interrupción para contar los pulsos en el pin 2
  attachInterrupt(digitalPinToInterrupt(pinSensor), contarPulsos, RISING);
}

void loop() {
  // Calcular el tiempo transcurrido
  unsigned long tiempoActual = millis();
  
  if (tiempoActual - tiempoAnterior >= intervaloTiempo) {
    // Calcular la velocidad angular
    velocidadAngular = (contadorPulsos * gradosPorIncremento) / (intervaloTiempo / 1000.0);

    // Imprimir la velocidad angular en la consola serie
    Serial.print("Velocidad Angular: ");
    Serial.print(velocidadAngular);
    Serial.println(" °/s");

    // Reiniciar el contador y el tiempo
    contadorPulsos = 0;
    tiempoAnterior = tiempoActual;
  }
}

void contarPulsos() {
  // Incrementar el contador de pulsos en cada flanco ascendente
  contadorPulsos++;
}
