//                               Gabriel Carrizales García - - -  - - - - - - - - - - - - - - - Carmen Guadalupe Hernández Fraire
//                                                            El Malcriado 1.0 - Sumo RC 2025
//                                                             Arduino Nano - L298N - 11.1v

#include <SoftwareSerial.h>

// Definir pines de dirección del motor
#define IN1 2   
#define IN2 3   
#define IN3 4   
#define IN4 5   

// Pines de velocidad PWM
#define ENA 9  // Motor izquierdo (PWM)
#define ENB 10 // Motor derecho (PWM)

// Configurar pines de comunicación serial UART
#define RX_PIN 6
#define TX_PIN 7

// Pines para los LEDs
#define LED_ROJO A4
#define LED_RGB  A5

SoftwareSerial Bluetooth(RX_PIN, TX_PIN); // Comunicación Bluetooth

// Definir velocidades
#define MAX_SPEED 255  // Velocidad máxima de los motores
#define GIROS_PROPIO_EJE 255 // Velocidad para giros en su propio eje
#define TURN_SPEED 50  // Velocidad reducida para giros suaves

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configurar pines de los LEDs
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_RGB, OUTPUT);
}

void loop() {
  if (Bluetooth.available() > 0) {
    char value = Bluetooth.read();
    Serial.println(value);

    if (value == 'F') {                 //F
      Forward();                        // Avanzar hacía Adelante
    } else if (value == 'B') {          //B
      Backward();                       // Avanzar hacía Atrás
    } else if (value == 'S') {          //S
      Stop();                           // Detenerse
    } else if (value == 'L') {          //L
      Left();                           // Girar hacía Izquierda sobre su propio eje
    } else if (value == 'R') {          //R
      Right();                          // Girar hacía Derecha sobre su propio eje
    } else if (value == 'G') {          //Q
      ForwardLeft();                    // Avanzar hacía Adelante mientras gira a la Izquierda
    } else if (value == 'I') {          //E
      ForwardRight();                   // Avanzar hacía Adelante mientras gira a la Derecha
    } else if (value == 'H') {          //Z
      BackLeft();                       // Avanzar hacía Atrás mientras gira a la Izquierda
    } else if (value == 'J') {          //C
      BackRight();                      // Avanzar hacía Atrás mientras gira a la Derecha
    } else if (value == 'W') {          //M
      digitalWrite(LED_ROJO, HIGH);     // Encender LED rojo
    } else if (value == 'w') {          //m
      digitalWrite(LED_ROJO, LOW);      // Apagar LED rojo
    } else if (value == 'U') {          //N
      digitalWrite(LED_RGB, HIGH);      // Encender LED camaleón
    } else if (value == 'u') {          //n
      digitalWrite(LED_RGB, LOW);       // Apagar LED camaleón
    }
  }
}

void Forward() {
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Backward() {
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Left() {
  analogWrite(ENA, GIROS_PROPIO_EJE);
  analogWrite(ENB, GIROS_PROPIO_EJE);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Right() {
  analogWrite(ENA, GIROS_PROPIO_EJE);
  analogWrite(ENB, GIROS_PROPIO_EJE);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void ForwardLeft() {
  analogWrite(ENB, TURN_SPEED);  
  analogWrite(ENA, GIROS_PROPIO_EJE);   
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void ForwardRight() {
  analogWrite(ENB, GIROS_PROPIO_EJE);   
  analogWrite(ENA, TURN_SPEED);  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void BackLeft() {
  analogWrite(ENB, TURN_SPEED);
  analogWrite(ENA, GIROS_PROPIO_EJE);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void BackRight() {
  analogWrite(ENB, GIROS_PROPIO_EJE);
  analogWrite(ENA, TURN_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
