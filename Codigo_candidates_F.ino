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

AF_DCMotor MDD(1); //motor delantero derechao
AF_DCMotor MDI(2); // motor delanter izq
AF_DCMotor MTD(3); // motor trasero derecho 
AF_DCMotor MTI(4); // motor trasero izq


// pines para recibir info
int inf = 35;
int infR = 53;

// declaracion de servos 
Servo claw; // garra
Servo rgb; // rgb 

MPU6050 mpu; // Nombre de mpu mas corto 
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
  rgb.write(0);

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
    Serial.println("MPU6050 conectado correctamente");
    calibrarMPU6050();
  } else {
    Serial.println("Error de conexión con MPU6050");
  }

  // Set up de RGB's
  tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X); // Shorter integration time
  selectChannel(SENSOR1_CHANNELL);
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found on Sensor L");
    while (1);
  }
  
}


void inicializarTCS34725(int channel) {
  selectChannel(channel);
  if (!tcs.begin()) {
    Serial.print("Error de conexión en canal: ");
    Serial.println(channel);
    while (1);
  }
}

void leerTCS34725() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if ((17 > r > 13) && (24 > g > 20) && (11 > b > 7) && (50 > c > 46)) return "Verde";
  else if ((24 > r > 20) && (7 > g > 3) && (6 > b > 2) && (33 > c > 29)) return "Rojo";
  else if ((52 > r > 48) && (37 > g > 33) && (14 > b > 10) && (107 > c > 103)) return "Amarillo";
  else if ((24 > r > 20) && (7 > g > 3) && (6 > b > 2) && (33 > c > 29)) return "Rosa";
  else if ((35 >r > 30) && (32 > g > 28) && (21 > b > 17) && (87 >c > 83)) return "Blanco";
  else if ((10 > r > 6) && (10 > g > 6) && (16 > b > 12) && (33 > c > 29)) return "Púrpura";
  else if ((14 > r > 10) && (15 > g > 11) && (13 > b > 9) && (32 > c > 28)) return "Negro";
  else return "Desconocido";
}

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
  gx_offset = gx_sum / muestras;
  gy_offset = gy_sum / muestras;
  Serial.println("Calibración completa.");
}

void actualizarAngulo() {
  int16_t gx_raw, gy_raw, gz_raw;
  selectChannel(MPU6050_CHANNEL);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  float gx = (gx_raw - gx_offset) / 131.0;
  float gy = (gy_raw - gy_offset) / 131.0;
  float gz = (gz_raw - gz_offset) / 131.0;

  long current_time = millis();
  float dt = (current_time - last_time_update) / 1000.0;
  last_time_update = current_time;

  float gradosX = gx * dt;
  float gradosY = gy * dt;
  float gradosZ = gz * dt;

  grados_acumulados_x += gradosX;
  grados_acumulados_y += gradosY;
  grados_acumulados_z += gradosZ;

  Serial.print("Grados: X=");
  Serial.print(gradosX);
  Serial.print(" | Y=");
  Serial.print(gradosY);
  Serial.print(" | Z=");
  Serial.println(gradosZ);

  Serial.print("Grados acumulados: X=");
  Serial.print(grados_acumulados_x);
  Serial.print(" | Y=");
  Serial.print(grados_acumulados_y);
  Serial.print(" | Z=");
  Serial.println(grados_acumulados_z);
}

void girar(float anguloObjetivo, bool derecha) {
  // Reinicia la acumulación de grados en Z
  grados_acumulados_z = 0;
  grados_acumulados_x = 0;
  grados_acumulados_y = 0;

  // Comienza el giro en la dirección especificada
  if (derecha) {
    giro_derecha();
  } else {
    giro_izquierda();
  }

  while (abs(grados_acumulados_z) < anguloObjetivo) {
    actualizarAngulo();
  }
}



void inclinacion() {
  bool subiendo = true;
  bool centrado = false;
  long tiempo_centrado = 0;

  while (true) {
    actualizarAngulo();

    if (subiendo) {
      adelante();
      if (grados_acumulados_y > 25) {
        subiendo = false;
        centrado = true;
      }
    }
    else if (centrado) {
      // Mantener equilibrio
      if (abs(grados_acumulados_y) < 5) {
        detener();
        if (tiempo_centrado == 0) tiempo_centrado = millis();
        if (millis() - tiempo_centrado > 6000) {
          centrado = false;
          subiendo = false;
        }
      }
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

bool esNegro(int channel) {
  uint16_t r, g, b, c;
  selectChannel(channel);
  tcs.getRawData(&r, &g, &b, &c);
  return (r > 5 && r < 18 && g > 5 && g < 18 && b > 5 && b < 18);
}

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

void infrarrojo(int pin, int t) {
  int read = digitalRead(pin);

  if (read == HIGH) {
    //Serial.print("Sensor ");
    //Serial.print(t);
    Serial.println(": Objeto detectado");
    return false; 
    }
  else {
    //Serial.print("Sensor ");
    //Serial.print(t);
    Serial.println(": No hay objeto");
    return true;
  }
}

void adelante() {
  MDD.setSpeed(120); 
  MDD.run(FORWARD); 
  MDI.setSpeed(120); 
  MDI.run(FORWARD);
  MTD.setSpeed(120);
  MTD.run(FORWARD);
  MTI.setSpeed(120);
  MTI.run(FORWARD);
}

void atras() {
  MDD.setSpeed(120);
  MDD.run(BACKWARD);
  MDI.setSpeed(120);
  MDI.run(BACKWARD);
  MTD.setSpeed(120);
  MTD.run(BACKWARD);
  MTI.setSpeed(120);
  MTI.run(BACKWARD);
}

void giro_derecha() {
  MDD.setSpeed(220);
  MDD.run(BACKWARD);
  MDI.setSpeed(220);
  MDI.run(FORWARD);
  MTD.setSpeed(220);
  MTD.run(BACKWARD);
  MTI.setSpeed(220);
  MTI.run(FORWARD);
}

void giro_izquierda() {
  MDD.setSpeed(220);
  MDD.run(FORWARD);
  MDI.setSpeed(220);
  MDI.run(BACKWARD);
  MTD.setSpeed(220);
  MTD.run(FORWARD);
  MTI.setSpeed(220);
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

  digitalWrite(v1,HIGH);
  digitalWrite(v2,HIGH);
  actualizarAngulo();
  //delay(50);
  infrarrojo(inf, 1);
  //delay(50);
  infrarrojo(infR, 2);
  //delay(50);
  leerTCS34725();
  inclinacion();

  long dist = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);

  Serial.print("Distancia Frontal: ");
  Serial.print(dist);
  Serial.print(" cm | Distancia Izquierda: ");
  Serial.print(distL);
  Serial.print(" cm | Distancia Derecha: ");
  Serial.print(distR);
  Serial.println(" cm");

  //if ((dist < 8) && (distR < 10)) {
    //girar(73, false);
  //} else if ((dist < 8) && (distL < 10)) {
    //girar(73, true);
  //} else if ((dist < 8) && (distL < 10) && (distR < 10)){
    //claw.write(120);
    //delay(1000);
    //atras();
    //delay(1000);
    //girar(73, true);
  //} else if ((distL < 10) && (distR < 10)){
    //adelante();
  //}  else if (dist < 8) {
    //  girar(73, true);
  //} else {
    //adelante();
  //}
}

  //if (esNegro(SENSOR1_CHANNELL) && !esNegro(infR)) {
    //giro_izquierda();
    //delay(250);
    //detener();
  //} else if (!esNegro(SENSOR1_CHANNELL) && esNegro(infR)) {
    //giro_derecha();
    //delay(250);
    //detener();
  //} else if (!esNegro(SENSOR1_CHANNELL) && !esNegro(infR)) {
    //adelante();
    //delay(100);
    //detener();
  //} else if (esNegro(SENSOR1_CHANNELL) && esNegro(infR)) {
    //adelante();
    //delay(100);
    //detener();
  //}
//}  
