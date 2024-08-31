#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27,16,2);  //

void setup() {
  // Inicializar el LCD
  lcd.init();
  
  //Encender la luz de fondo.
  lcd.backlight();
  
}

void loop() {
  int sen1=analogRead(A0);
  float sen2=analogRead(A1)*(5.0 / 1023.0);
  float sen3=analogRead(A2)*(100.0 / 1023.0);
  int tiempo=millis()/1000;
  // Cursor en la primera posición de la primera fila
  lcd.setCursor(0,0);
  lcd.print("ADC:");
  lcd.print(sen1);
  lcd.print("   ");
  // Cursor en la 11° posición de la primera fila
  lcd.setCursor(10,0);
  lcd.print("V:");
  lcd.print(sen2,1);//1 decimal
  lcd.print("V  ");
  // Cursor en la primera posición de la 2° fila
  lcd.setCursor(0,1);
  lcd.print("T:");
  lcd.print(sen3,1); //1 decimal
  lcd.print("337C  "); // "337" -> "°"
  // Cursor en la 11° posición de la 2° fila
  lcd.setCursor(10,1);
  lcd.print("t:");
  lcd.print(tiempo);
  lcd.print(" s  ");
  
  delay(200);
  
}