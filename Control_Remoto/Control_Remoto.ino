/*Control remoto con Gamepad Shield para Arduino, comunicacion con el chip NRF2401L.
  Autor: Javier Vargas. El Hormiguero.
  https://creativecommons.org/licenses/by/4.0/
*/

//Pines
#define Pin_xJoystic A0
#define Pin_yJoystic A1
#define Pin_BotonA 2
#define Pin_BotonB 3
#define Pin_BotonC 4
#define Pin_BotonD 5

//CONFIGURACION
#define V3_3 //Módulo conectado a 3.3V (descomentar en caso contrario)
#define RangoJoystic 100 //Rango de salida de lectura del joystic
#define OffsetX 7
#define OffsetY 0

//Variable con la dirección del canal por donde se va a transmitir
byte direccion[5] = {'h', 'o', 'r', 'm', 'i'};

//NRF24L01
#define CE_PIN 9
#define CSN_PIN 10
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(CE_PIN, CSN_PIN);

//Variables del mando
int MaxADC = 1023;
float datos[2];

void setup() {

  Serial.begin(115200);

  //PinMode
  pinMode(Pin_BotonA, INPUT_PULLUP);
  pinMode(Pin_BotonB, INPUT_PULLUP);
  pinMode(Pin_BotonC, INPUT_PULLUP);
  pinMode(Pin_BotonD, INPUT_PULLUP);

  //inicializamos el NRF24L01
  radio.begin();
  radio.openWritingPipe(direccion);

  //Entrada analógica máxima
#ifdef V3_3
  MaxADC = (float)(3.33 / 5) * 1023;
#else
  MaxADC = 1023;
#endif
}

void loop() {
  datos[0] = JoysticX();
  datos[1] = JoysticY();
  radio.write(datos, sizeof(datos));
  Serial.print(datos[0]);
  Serial.print(" / ");
  Serial.println(datos[1]);
  delay(100);
}

int JoysticX() {
  int x_Joystic;
  x_Joystic = map(analogRead(Pin_xJoystic) + OffsetX, 0, MaxADC, -RangoJoystic, RangoJoystic);
  x_Joystic = constrain(x_Joystic, -RangoJoystic, RangoJoystic);
  return x_Joystic;
}

int JoysticY() {
  int y_Joystic;
  y_Joystic = map(analogRead(Pin_yJoystic) + OffsetY, 0, MaxADC, -RangoJoystic, RangoJoystic);
  y_Joystic = constrain(y_Joystic, -RangoJoystic, RangoJoystic);
  return y_Joystic;
}

boolean BotonA() {
  return !digitalRead(Pin_BotonA);
}

boolean BotonB() {
  return !digitalRead(Pin_BotonB);
}

boolean BotonC() {
  return !digitalRead(Pin_BotonC);
}

boolean BotonD() {
  return !digitalRead(Pin_BotonD);
}



