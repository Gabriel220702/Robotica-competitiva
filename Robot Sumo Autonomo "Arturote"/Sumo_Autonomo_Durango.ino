#include <NewPing.h>
#include <PID_v1.h>

// Definiciones de pines para los motores
#define ENA 11
#define ENB 9
#define IN1 8
#define IN2 10
#define IN3 12
#define IN4 13

// Definiciones de pines para los sensores ultrasónicos
#define TRIGGER_PIN_FRONT 2
#define ECHO_PIN_FRONT 3
#define TRIGGER_PIN_REAR 4
#define ECHO_PIN_REAR 5
#define MAX_DISTANCE 200

// Definiciones de pines para los seguidores de línea
#define LINE_SENSOR_FL A0 // Front Left
#define LINE_SENSOR_FR A1 // Front Right
#define LINE_SENSOR_RL A2 // Rear Left
#define LINE_SENSOR_RR A3 // Rear Right

// Pines para LED y buzzer
#define LED_PIN 6
#define BUZZER_PIN 7

// Creación de objetos de sensor ultrasónico
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRear(TRIGGER_PIN_REAR, ECHO_PIN_REAR, MAX_DISTANCE);

// Variables de control de motores
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2.0, 5.0, 1.0, DIRECT); // Parámetros Kp, Ki, Kd

void setup() {
  Serial.begin(9600);

  // Configuración de pines de motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuración de pines para seguidores de línea
  pinMode(LINE_SENSOR_FL, INPUT);
  pinMode(LINE_SENSOR_FR, INPUT);
  pinMode(LINE_SENSOR_RL, INPUT);
  pinMode(LINE_SENSOR_RR, INPUT);

  // Configuración de pines de LED y buzzer
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Inicializar el PID
  Setpoint = 25;  // La distancia deseada del objeto en cm
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Limites de salida para PWM
}

void loop() {
  unsigned int distanceFront = sonarFront.ping_cm();
  unsigned int distanceRear = sonarRear.ping_cm();
  
  int lineFL = analogRead(LINE_SENSOR_FL);  // Leer seguidor de línea frontal izquierdo
  int lineFR = analogRead(LINE_SENSOR_FR);  // Leer seguidor de línea frontal derecho
  int lineRL = analogRead(LINE_SENSOR_RL);  // Leer seguidor de línea trasero izquierdo
  int lineRR = analogRead(LINE_SENSOR_RR);  // Leer seguidor de línea trasero derecho
  
  Serial.print("Front Distance: ");
  Serial.print(distanceFront);
  Serial.print(" cm - Rear Distance: ");
  Serial.print(distanceRear);
  Serial.print(" cm - Line FL: ");
  Serial.print(lineFL);
  Serial.print(" - Line FR: ");
  Serial.print(lineFR);
  Serial.print(" - Line RL: ");
  Serial.print(lineRL);
  Serial.print(" - Line RR: ");
  Serial.println(lineRR);

  // Control PID para mantener la distancia
  if (distanceFront > 0) { // Evitar que 0 (sin lectura) afecte el control
    Input = distanceFront;
    myPID.Compute();
    analogWrite(ENA, Output);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  if (distanceRear > 0) { // Control independiente para el motor trasero, ejemplo básico
    analogWrite(ENB, 255 - Output); // Ejemplo inverso
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // Aquí puedes añadir condiciones basadas en los valores de los seguidores de línea
  // Por ejemplo, encender un LED o activar un buzzer si se detecta una línea
  if (lineFL > 500) { // Suponiendo que 500 es el umbral para detectar línea
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(100);  // Retraso para la estabilidad del lazo de control
}
