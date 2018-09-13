/* Robot auto equilibrado con dos PID en cascada y un control de la posición.
   Autor: Javier Vargas. El Hormiguero.
   https://creativecommons.org/licenses/by/4.0/
*/
//PINES
#define PinSTEP1 8
#define PinSTEP2 5
#define PinDIR1 9
#define PinDIR2 6
#define PinENABLE1 7
#define PinENABLE2 4
#define CE_PIN A0
#define CSN_PIN A1

//CONFIGURACION
//General
#define AnguloOFF 60 //Grados entorno a la vertical para desactivado de motores
#define AnguloOK 1 //Grados entorno a la vertical para activado de motores
//Cinemática
#define RadioRueda 3.25f //(cm) //Radio de la rueda
#define PasosVuelta 3200 //Pasos por vuelta del motor paso a paso
#define velocidadMax 120 //(cm/s)
#define velocidadMin 0.0001 //(cm/s)
#define DIRECCION1 HIGH //Rueda uno directa
#define DIRECCION2 LOW //Rueda dos invertida
//Control
#define Muestreo 25 //(ms) Periodo del algoritmo de control
#define Offset 4.5f //Offset del acelerometro
#define KfiltComp 0.99f //Constante del filtro complementario del acelerómetro y giróscopo
#define KfiltroRFvel 0.2 //Filtro del valor de velocidad obtenido por RF
#define KfiltroRFgiro 0.8 //Filtro del valor de giro obtenido por RF
#define TiempoControlX 1500 //(ms) Tiempo sin recibir por RF tras el que se activa el control de posicion 
double KpA = 17, KiA = 0, KdA = 3.5;  //Control PID del angulo --> Sin contrapeso mejor: KpA = 20, KiA = 0, KdA = 5;
double KpV = 0.15, KiV = 0.01, KdV = 0; //Control PID de la velocidad --> Sin contrapeso mejor: KpV = 0.12, KiV = 0.04, KdV = 0;
double KpX = 0.8; //Control proporcional de la posicion
float outMaxA = 400; //Acelercion maxima a la salida de PID del angulo
float outMaxV = 20; //Angulo maximo objetivo a la salida del PID de velocidad
float outMaxX = 30; //Velocidad objetivo máxima del control de posicion

//LIBRERIAS Y VARIABLES

//PID
#include <PID_v1.h>
double SetpointA, InputA, OutputA;
PID pidA(&InputA, &OutputA, &SetpointA, KpA, KiA, KdA, DIRECT);

double SetpointV, InputV, OutputV;
PID pidV(&InputV, &OutputV, &SetpointV, KpV, KiV, KdV, DIRECT);

//RADIOFRECUENCIA
byte direccion[5] = {'h', 'o', 'r', 'm', 'i'}; //Variable con la dirección del canal por donde se va a transmitir
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(CE_PIN, CSN_PIN);

//IMU
#include <MPU6050_CompFilter.h>
MPU6050_CompFilter mpu(0x68);

//TIMER
#include <TimerOne.h>
#include <TimerThree.h>

#include <digitalWriteFast.h> //Más rapido!
unsigned long m1, m2 = 0;
float Angulo = 0; //Angulo del robot
float AnguloObj = 0; //Angulo objetivo
float Velocidad = 0; //Velocidad del robot
float VelocidadObj = 0; //Velocidad objetivo
float Posicion = 0; //Posicion del robot
float PosicionObj = 0; //Posicion objetivo
float Aceleracion = 0; //Aceleracion del robot
float Giro = 0; //Nivel de giro del robot
float PPcm;
float datos[2];
unsigned long TimeMov = 0;

void setup() {

  //PinMode
  pinMode(PinSTEP1, OUTPUT);
  pinMode(PinSTEP2, OUTPUT);
  pinMode(PinDIR1, OUTPUT);
  pinMode(PinDIR2, OUTPUT);
  pinMode(PinENABLE1, OUTPUT);
  pinMode(PinENABLE2, OUTPUT);

  digitalWrite(PinENABLE1, HIGH);
  digitalWrite(PinENABLE2, HIGH);
  digitalWrite(PinDIR1, DIRECCION1);
  digitalWrite(PinDIR2, DIRECCION2);

  //IniciO IMU
  mpu.Iniciar((float)Muestreo / 1000);
  mpu.setKcompFilter(KfiltComp);

  //Inicio PID
  pidA.SetTunings(KpA, KiA, KdA);
  pidA.SetOutputLimits(-outMaxA, outMaxA);
  pidA.SetMode(AUTOMATIC);

  pidV.SetTunings(KpV, KiV, KdV);
  pidV.SetOutputLimits(-outMaxV, outMaxV);
  pidV.SetMode(AUTOMATIC);

  //inicio NRF24L01
  radio.begin();
  radio.openReadingPipe(1, direccion);
  radio.startListening();   //empezamos a escuchar por el canal

  //Inicio TIMER
  Timer1.initialize(250000);
  Timer1.attachInterrupt(Step1);
  Timer1.stop();

  Timer3.initialize(250000);
  Timer3.attachInterrupt(Step2);
  Timer3.stop();

  PPcm = (float)PasosVuelta / (2 * PI * RadioRueda); //Pasos por centímetro recorrido

  delay(1000);
}

////////////////////
////////LOOP////////
////////////////////

void loop() {
  //Loop cada periodo de muestreo
  if (m1 != millis() / Muestreo) {
    m1 = millis() / Muestreo;

    ObtenerAngulo(); //Lectura de la IMU

    //ROBOT EN POSICION VERTICAL
    if (RobotVerticalOK()) {
      ControlRemoto(); //Recibe la velocidad y el giro por RF
      ControlPosicion();  //Tras cierto tiempo sin recibir orden por RF mantiene la posicion
      ControlVelocidad(); //PID para ajustar el angulo objetivo y mantener la velocidad
      ControlEquilibrio(); //PID para ajustar la aceleracion de las ruedas y mantener el equilibrio
    }

    //ROBOT HORIZONTAL
    else {
      MotoresOFF(); //Apaga los motores 
      ReiniciarPID(); //Reinicia los controladores PID
    }

  }
}

////////////////////
////////////////////
////////////////////

void ReiniciarPID() {
  pidA.SetOutputLimits(0.0, 1.0);  //Fuerza el máximo a 0
  pidA.SetOutputLimits(-1.0, 0.0);  //Fuerza el minimo a 0
  pidA.SetOutputLimits(-outMaxA, outMaxA);
  pidV.SetOutputLimits(0.0, 1.0);  //Fuerza el máximo a 0
  pidV.SetOutputLimits(-1.0, 0.0);  //Fuerza el minimo a 0
  pidV.SetOutputLimits(-outMaxV, outMaxV);
}

void MotoresOFF() {
  Velocidad = 0;
  Posicion = 0;
  PosicionObj = 0;
  SetVelocidad(0, 0);
}

boolean RobotVerticalOK() {
  static boolean VerticalOK = 0;

  //El robot se encuentra en posicion correcta
  if (VerticalOK) {
    if (abs(Angulo) > AnguloOFF) {
      //Angulo de caida superado
      VerticalOK = 0;
    }
  }

  //El robot no se encuentra totalmente vertical
  else {
    if (abs(Angulo) < AnguloOK) {
      //Angulo de posicion correcta superado
      VerticalOK = 1;
    }
  }

  return VerticalOK;
}

void ControlRemoto() {
  static float datosFilt[2];

  if (radio.available()) {
    //Recepcion por RF
    radio.read(datos, sizeof(datos));
    //Filtrado de los datos
    datosFilt[1] = (KfiltroRFvel * datos[1] + (1 - KfiltroRFvel) * datosFilt[1]);
    datosFilt[0] = (KfiltroRFgiro * datos[0] + (1 - KfiltroRFgiro) * datosFilt[0]);
    //Interpretacion de velocidad y giro
    VelocidadObj = datosFilt[1] / 2;
    Giro = datosFilt[0] / 5;
    //Actualizacion del contador (quita el control de posicion)
    if (abs(VelocidadObj) > 5 || abs(Giro) > 2) TimeMov = millis();
  }
}

void ObtenerAngulo() {
  //Lectura del acelerometro
  mpu.Lectura(1, 0); //Eje x
  Angulo = mpu.angX() + Offset;
}

void ControlPosicion() {
  static boolean e = 0;
  if (millis() > TimeMov + TiempoControlX) {
    if (!e) PosicionObj = Posicion, e = 1; //Marcamos la posicion a mantener
    VelocidadObj = KpX * (PosicionObj - Posicion); //Control proporcional de la posicion tras cierto tiempo sin recibir RF 
    VelocidadObj = constrain(VelocidadObj, -outMaxX, outMaxX);
    Giro = 0;
  }
  else if (e) e = 0;
}

void ControlVelocidad() {
  //Control PID del angulo objetivo a partir de la velocidad
  SetpointV = VelocidadObj; //Consigna la velocidad objetivo
  InputV = Velocidad; //Entrada la velocidad objetivo
  pidV.Compute();
  AnguloObj = -OutputV; //La salida es el angulo objetivo del siguiente PID
}

void ControlEquilibrio() {
  //Control PID del angulo
  SetpointA = AnguloObj; //Consigna el angulo objetivo
  InputA = Angulo; //Entrada el angulo del robot
  pidA.Compute();
  Aceleracion = OutputA; //Salida la aceleracion

  //Calculo de la velocidad
  Velocidad += Aceleracion * (float)Muestreo / 1000;

  //Posicion del robot
  Posicion += Velocidad * (float)Muestreo / 1000;

  //Salida aplicada a la velocidad de las ruedas
  float v1 = Velocidad + Giro; //Velocidad rueda izquierda
  float v2 = Velocidad - Giro; //Velocidad rueda derecha
  SetVelocidad(v1, v2);
}

void SetVelocidad(float v1, float v2) {
  static volatile boolean e1, e2 = 0; //Estado de los timers
  static volatile boolean d1 = 1;
  static volatile boolean d2 = 1; //Direccion de los motores

  //Limite de velocidad entre los limites de velocidad maxima
  v1 = constrain(v1, -velocidadMax, velocidadMax);
  v2 = constrain(v2, -velocidadMax, velocidadMax);

  //Direccion motor 1
  if (v1 >= 0) {
    if (!d1) {
      d1 = 1;
      digitalWriteFast(PinDIR1, DIRECCION1);
    }
  }
  else if (d1) {
    d1 = 0;
    digitalWriteFast(PinDIR1, !DIRECCION1);
  }

  //Direccion motor 2
  if (v2 >= 0) {
    if (!d2) {
      d2 = 1;
      digitalWriteFast(PinDIR2, DIRECCION2);
    }
  }
  else if (d2) {
    d2 = 0;
    digitalWriteFast(PinDIR2, !DIRECCION2);
  }

  //Velocidad motor 1
  if (abs(v1) >= velocidadMin) {
    unsigned long T1 = 500000 / (abs(v1) * PPcm); //Periodo de un semipulso
    if (!e1) { //Inicia el timer
      Timer1.start();
      digitalWriteFast(PinENABLE1, LOW); //Activa el motor
      e1 = 1;
    }
    Timer1.setPeriod(T1); //Periodo de la interrpucion
  }
  else if (e1) { //Desactiva el timer
    Timer1.stop();
    digitalWriteFast(PinENABLE1, HIGH); //Desactiva el motor
    e1 = 0;
  }

  //Velocidad motor 2
  if (abs(v2) >= velocidadMin) {
    unsigned long T2 = 500000 / (abs(v2) * PPcm); //Periodo de un semipulso
    if (!e2) {  //Inicia el timer
      Timer3.start();
      digitalWriteFast(PinENABLE2, LOW); //Activa el motor
      e2 = 1;
    }
    Timer3.setPeriod(T2); //Periodo de la interrpucion
  }
  else if (e2) { //Desactiva el timer
    Timer3.stop();
    digitalWriteFast(PinENABLE2, HIGH); //Desactiva el motor
    e2 = 0;
  }

}

//STEP

void Step1() {
  static volatile boolean e = 0;
  if (e) {
    digitalWriteFast(PinSTEP1, LOW);
    e = 0;
  }
  else {
    digitalWriteFast(PinSTEP1, HIGH);
    e = 1;
  }
}

void Step2() {
  static volatile boolean e = 0;
  if (e) {
    digitalWriteFast(PinSTEP2, LOW);
    e = 0;
  }
  else {
    digitalWriteFast(PinSTEP2, HIGH);
    e = 1;
  }
}

