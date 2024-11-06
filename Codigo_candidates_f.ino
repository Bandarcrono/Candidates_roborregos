#include <AFMotor.h> // Libreria para el adafruit 
#include <Wire.h> // Libreria para el multiplexor  
#include <Adafruit_TCS34725.h> // Libreria para el RGB
#include <MPU6050.h> // Libreria para el giroscoipio
#include <Servo.h> // Libreria para programar servos

#define MUX_ADDRESS 0x70 // Conexion para el multiplexor tipical "0x70" SDA/SCL
#define SENSOR1_CHANNELL 6 // SD6 y SC6
#define MPU6050_CHANNEL 7 // SD7 y SC7

//Ultrasonic declaraciones
#define TRIG_PIN_R 49 
#define ECHO_PIN_R 48
#define TRIG_PIN 51
#define ECHO_PIN 50
#define TRIG_PIN_L 27
#define ECHO_PIN_L 26

AF_DCMotor MDD(1); // motor delantero derecho
AF_DCMotor MDI(2); // motor delanter izq
AF_DCMotor MTD(3); // motor trasero derecho 
AF_DCMotor MTI(4); // motor trasero izq

//infrarrojos
int inf = 35;
int infR = 53;

// contador de garra
int cont=0;

// declaraciones 
Servo claw; // garra
Servo rgb; // rgb 

MPU6050 mpu; // mpu  
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); // lectura cada 2.4ms , con nu gain de 1X (precision(/sensibilidad del led)) 

float gx_offset = 0, gy_offset = 0, gz_offset =0; // declaracion de varaibles para el giroscopio (mpu)
float gradosX=0, gradosY=0, gradosZ=0;
float grados_acumulados_x = 0, grados_acumulados_y = 0, grados_acumulados_z = 0;
long last_time_update = 0; 


// function para iniciar multiplexor
void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  last_time_update = millis();
  Wire.begin(); // multiplexor serial
  Serial.begin(9600);

  // Set up del multiplexor
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  // Set up de motores
  MDD.setSpeed(255); MDI.setSpeed(255); MTD.setSpeed(255); MTI.setSpeed(255);
  MDD.run(RELEASE); MDI.run(RELEASE); MTD.run(RELEASE); MTI.run(RELEASE);

  // Set up servos
  claw.attach(10);
  rgb.attach(9);
  claw.write(0);
  rgb.write(180);

  // Set up infrarrojos
  pinMode(inf, INPUT);
  pinMode(infR, INPUT);

  // Set up de ultrasónicos
  pinMode(TRIG_PIN_R, OUTPUT); pinMode(ECHO_PIN_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_L, OUTPUT); pinMode(ECHO_PIN_L, INPUT);

  // Set up del giroscopio
  selectChannel(MPU6050_CHANNEL);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado");
    calibrarMPU6050();
  } else {
    Serial.println("Error con MPU6050");
  }

  // Set up de RGB's
  tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); // Shorter integration time
  selectChannel(SENSOR1_CHANNELL);
  if (!tcs.begin()) {
    Serial.println("Error TCS34725");
    while (1);
  }
  
}

// Lecturas multiplexor
void inicializarTCS34725(int channel) {
  selectChannel(channel);
  if (!tcs.begin()) {
    Serial.print("Error de conexión en canal: ");
    Serial.println(channel);
    while (1);
  }
}

bool negroizq() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 1 && r < 7) && (g >= 1 && g < 6) && (b >= 1 && b < 7))
    //if ((0<r<7) && (0<g<7) && (0<b<7))
    {
      Serial.println("True");
      return true;
    } else {
      Serial.println("False");
      return false;
  }
}  

bool verde() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 6 && r <= 10) && (g >= 9 && g <= 14) && (b >= 3 && b <= 6))
    {
      //Serial.println("True");
      return true;
    } else {
      //Serial.println("False");
      return false;
  }
}

bool rojo() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 3 && r <= 7) && (g >= 0 && g <= 2) && (b >= 0 && b <= 2))
    {
      //Serial.println("True");
      return true;
    } else {
      //Serial.println("False");
      return false;
  }
}

bool amarillo() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 11 && r <= 15) && (g >= 7 && g <= 11) && (b >= 1 && b <= 4))
    {
      //Serial.println("True");
      return true;
    } else {
      //Serial.println("False");
      return false;
  }
}

bool morado() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 2 && r <= 6) && (g >= 2 && g <= 6) && (b >= 4 && b <= 8))
    {
      //Serial.println("True");
      return true;
    } else {
      //Serial.println("False");
      return false;
  }
}

bool rosa() {
  uint16_t r, g, b, c;
  selectChannel(SENSOR1_CHANNELL);
  tcs.getRawData(&r, &g, &b, &c);
    if ((r >= 5 && r <= 7) && (g >= 1 && g <= 3) && (b >= 2 && b <= 4))
    {
      //Serial.println("True");
      return true;
    } else {
      //Serial.println("False");
      return false;
  }
} 

// Funcion para identificar los colores
void led(){ // 1 vez prende 
  if (verde()==true){
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);

  }
  else if(morado()==true){ // 2 vez prende
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  }

  else if(rosa()==true) // 3 veces prende
  {
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  }
  else if(amarillo()==true){ // 4 veces prende 
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  digitalWrite(green_led,HIGH);
  delay(250);
  digitalWrite(green_led,LOW);
  delay(250);
  }
}

// Funcion para calibrar mpu
void calibrarMPU6050() {
  int16_t gx, gy, gz;
  long gx_sum = 0, gy_sum = 0;
  int muestras = 2000;

  for (int i = 0; i < muestras; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    delay(2);
  }
  // Promedio de lecturas
  gx_offset = gx_sum / muestras;
  gy_offset = gy_sum / muestras;
  Serial.println("Calibración completa.");
}

// Funcion para obtener cambios en angulos en cada eje
void actualizarAngulo() {
  int16_t gx_raw, gy_raw, gz_raw;
  selectChannel(MPU6050_CHANNEL);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // Funciones para lecturas precisas en grados/s
  float gx = (gx_raw - gx_offset) / 131.0;
  float gy = (gy_raw - gy_offset) / 131.0;
  float gz = (gz_raw - gz_offset) / 131.0;

  long current_time = millis();
  float dt = (current_time - last_time_update) / 1000.0;
  last_time_update = current_time;

  //Cambio en grados
  float gradosX = gx * dt;
  float gradosY = gy * dt;
  float gradosZ = gz * dt;

  grados_acumulados_x += gradosX;
  grados_acumulados_y += gradosY;
  grados_acumulados_z += gradosZ;

  //Serial.print("Grados: X=");
  //Serial.print(gradosX);
  // Serial.print(" | Y=");
  //Serial.print(gradosY);
  //Serial.print(" | Z=");
  //Serial.println(gradosZ);

  //Serial.print("Grados acumulados: X=");
  //Serial.print(grados_acumulados_x);
  //Serial.print(" | Y=");
  //Serial.print(grados_acumulados_y);
  //Serial.print(" | Z=");
  //Serial.println(grados_acumulados_z);
}

// Funcion para dar vueltas mas precisas
void girar(float anguloObjetivo, bool derecha) {
  // Reiniciar grados en Z
  grados_acumulados_z = 0;

  if (derecha) {
    giro_derecha();
  } else {
    giro_izquierda();
  }

  while (abs(grados_acumulados_z) < anguloObjetivo) {
    actualizarAngulo();
  }
}


// Funcion para el balancin
void inclinacion() {
  bool subir = true;
  bool centrar = false;
  long tiempo_centro = 0;

  // Repeticion del loop hasta balancear
  while (true) {
    actualizarAngulo();

    //Avanza hasta un cambio de grados
    if (subir) {
      adelante();
      delay(4500);
      if (grados_acumulados_y) > 20){
        subir = false;
        centrar = true;
      }
    }

    //Si esta en el centro comienza conteo
    else if (centrar) {
      // Mantener equilibrio
      if (abs(grados_acumulados_y) < 5) {
        detener();
        if (tiempo_centrado == 0) tiempo_centrado = millis();
        if (millis() - tiempo_centrado > 6000) {
          centrado = false;
          subiendo = false;
        }
      }

      // Balanceo hasta quedar en centro
      else {
        if (grados_acumulados_y > 5) atras();
        else if (grados_acumulados_y < -5) adelante();
      }
    }

    else {
      adelante();
      if (grados_acumulados_y < -10) break;
    }
    delay(50);
  }
  detener();
}

// Funcion ultrasonicos
long obtenerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Distancia en cm
  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2; 
  return distancia;
}

// Funcion infrarrojos
bool infrarrojo(int pin, int t) {
  int read = digitalRead(pin);

  if (read == HIGH) {
    //Serial.print("Sensor ");
    //Serial.print(t);
    //Serial.println(": Objeto detectado");
    return true; 
    }
  else {
    //Serial.print("Sensor ");
    //Serial.print(t);
    //Serial.println(": No hay objeto");
    return false;
  }
}

void adelante() {
  MDD.setSpeed(180); 
  MDD.run(FORWARD); 
  MDI.setSpeed(180); 
  MDI.run(FORWARD);
  MTD.setSpeed(180);
  MTD.run(FORWARD);
  MTI.setSpeed(180);
  MTI.run(FORWARD);
}

void atras() {
  MDD.setSpeed(180);
  MDD.run(BACKWARD);
  MDI.setSpeed(180);
  MDI.run(BACKWARD);
  MTD.setSpeed(180);
  MTD.run(BACKWARD);
  MTI.setSpeed(180);
  MTI.run(BACKWARD);
}

void giro_derecha() {
  MDD.setSpeed(180); // 220 
  MDD.run(BACKWARD);
  MDI.setSpeed(180);
  MDI.run(FORWARD);
  MTD.setSpeed(180);
  MTD.run(BACKWARD);
  MTI.setSpeed(180);
  MTI.run(FORWARD);
}

void giro_izquierda() {
  MDD.setSpeed(200);
  MDD.run(FORWARD);
  MDI.setSpeed(200);
  MDI.run(BACKWARD);
  MTD.setSpeed(200);
  MTD.run(FORWARD);
  MTI.setSpeed(200);
  MTI.run(BACKWARD);
}

void detener() {
  MDD.setSpeed(0);
  MDD.run(RELEASE);
  MDI.setSpeed(0);
  MDI.run(RELEASE);
  MTD.setSpeed(0);
  MTD.run(RELEASE);
  MTI.setSpeed(0);
  MTI.run(RELEASE);
}

void loop() {

  actualizarAngulo();
  infrarrojo(infR, 2);
  //inclinacion();
  negroizq();
  morado();
  verde();
  rojo();
  amarillo();
  rosa();
  led();

  long dist = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);

  //Serial.print("Distancia Frontal: ");
  //Serial.print(dist);
  //Serial.print(" cm | Distancia Izquierda: ");
  //Serial.print(distL);
  //Serial.print(" cm | Distancia Derecha: ");
  //Serial.print(distR);
  //Serial.println(" cm");
//}
    //}
// Codigo Maze y Pelota
   /* if ((dist < 7) && (distR < 10)) {
      girar(73, false);
    } else if ((dist < 7) && (distL < 10)) {
      girar(73, true);
    } else if ((dist < 7) && (distL < 10) && (distR < 10)){
      girar(73, true);
      // claw.write(140);
      // atras();
      // delay(3000);
      // claw.write(0);
      girar(73, true);
    } else if ((distL < 10) && (distR < 10)){
      adelante();
    } else if (dist < 7) {
      girar(73, true);
    } else if (negroizq()==true) {
      atras();
      delay(1000);
      girar(73, true)
    } else {
      adelante();
    }
  //} */

// Codigo Linea
  if ((negroizq() == true) && (infrarrojo(infR,2) == false)) {
    giro_izquierda();
  } else if ((negroizq() == false) && (infrarrojo(infR,2) == true)) {
    giro_derecha();
  } else if ((negroizq() == false) && (infrarrojo(infR,2) == false )) {
    adelante();
  } else if ((negroizq() == true) && ((infrarrojo(infR,2) == true))) {
    adelante();
  }
}  
