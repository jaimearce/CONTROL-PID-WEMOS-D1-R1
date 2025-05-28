#include <Arduino.h>

// PID y control
float Kp = 1.5;
float Ki = 0.05;
float Kd = 0.3;
const float alpha = 0.1; // filtro de primer orden

float setpoint = 35.0;
float error = 0;
float error_anterior = 0;
float error_acumulado = 0;
float error_filtrado = 0;

const float ALTURA_MAXIMA = 50;//Cmts
const float ALTURA_MINIMA = 5;//Cmts
const float voltaje_min   = 0.4;//voltios
const float voltaje_max   = 5;//voltios

const float ConstWemos = 130;//divisor de voltaje vmax 3.3 voltios

const int PIN_SENSOR = A0;
const int PIN_BOMBA  = D2;

// Mapeo PWM (ajústalo según tu válvula si es necesario)
const int PWM_MAXIMO = 1000;
const int PWM_MINIMO = 100;

// Control de tiempo
unsigned long tiempo_anterior = 0;
const unsigned long INTERVALO_MUESTREO = 100; // ms
unsigned long tiempo_ultimo_print = 0;

// Filtro de promedio
#define FILTER_SIZE 5
float filtro[FILTER_SIZE];
byte filtro_index = 0;

// Estado
bool sistemaActivo = false;
String inputString = "";

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_BOMBA, OUTPUT);
  analogWriteFreq(1000); // PWM 1kHz

  float lectura_inicial = leerNivel();
  for (int i = 0; i < FILTER_SIZE; i++) filtro[i] = lectura_inicial;

  Serial.println("Sistema PID para control de nivel - Wemos D1 R1");
  Serial.println("Comandos: I = Iniciar, X = Detener, 35.5 = nuevo setpoint");
  Serial.println("También puedes cambiar Kp, Ki, Kd con: p3.25 | i0.04 | d0.2");
}

void loop() {
  procesarSerial();

  unsigned long tiempo_actual = millis();
  if (tiempo_actual - tiempo_anterior >= INTERVALO_MUESTREO && sistemaActivo) {
    tiempo_anterior = tiempo_actual;

    float altura_actual = lecturaFiltrada();
    float delta_t = INTERVALO_MUESTREO / 1000.0;

    // PID con filtro de primer orden
    error = setpoint - altura_actual;
    error_filtrado = alpha * error + (1 - alpha) * error_filtrado;
    error_acumulado += error * delta_t;
    float derivada = (error_filtrado - error_anterior) / delta_t;

    float salida_pid = Kp * error_filtrado + Ki * error_acumulado + Kd * derivada;

    // Limitamos la salida a 0-40 cm y la convertimos a PWM 0-100
    float salida_limitada = constrain(salida_pid, 0, 40);
    int pwm = map(salida_limitada, 0, 40, PWM_MINIMO, PWM_MAXIMO);

    analogWrite(PIN_BOMBA, pwm);

    // Imprimir por serial
    if (tiempo_actual - tiempo_ultimo_print >= 1000) {
      Serial.print("Tiempo: ");
      Serial.print(tiempo_actual / 1000.0, 2);
      Serial.print("s |Set point: ");
      Serial.print(setpoint, 2);
      Serial.print("cm | Altura: ");
      Serial.print(altura_actual, 2);
      Serial.print(" cm | PWM: ");
      Serial.print(pwm);
      Serial.print(" | PID: [Kp=");
      Serial.print(Kp, 2);
      Serial.print(" Ki=");
      Serial.print(Ki, 3);
      Serial.print(" Kd=");
      Serial.print(Kd, 2);
      Serial.println("]");
      tiempo_ultimo_print = tiempo_actual;
    }

    error_anterior = error_filtrado;
  }
}

// Filtro de lectura de sensor
float lecturaFiltrada() {
  float valor = leerNivel();
  filtro[filtro_index] = valor;
  filtro_index = (filtro_index + 1) % FILTER_SIZE;

  float suma = 0;
  for (int i = 0; i < FILTER_SIZE; i++) suma += filtro[i];
  return suma / FILTER_SIZE;
}

// Conversión de voltaje a altura
float leerNivel() {
  int lectura = analogRead(PIN_SENSOR);
  float voltaje_sensor = lectura / ConstWemos;
  float nivel = ((lectura / ConstWemos) * (ALTURA_MAXIMA-ALTURA_MINIMA)/(voltaje_max-voltaje_min));          
  return constrain(nivel, 0, ALTURA_MAXIMA);
}

// Procesamiento del monitor serial con cambio de Kp, Ki, Kd
void procesarSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputString.trim();

      if (inputString == "I" || inputString == "i") {
        sistemaActivo = true;
        error_acumulado = 0;
        error_anterior = 0;
        error_filtrado = 0;
        Serial.println("-> Sistema INICIADO <-");

      } else if (inputString == "X" || inputString == "x") {
        sistemaActivo = false;
        analogWrite(PIN_BOMBA, 0);
        Serial.println("-> Sistema DETENIDO <-");

      } else if (inputString.startsWith("p") || inputString.startsWith("P")) {
        float nuevo = inputString.substring(1).toFloat();
        if (nuevo >= 0) {
          Kp = nuevo;
          Serial.print("-> Nuevo Kp = ");
          Serial.println(Kp, 3);
        }

      } else if (inputString.startsWith("i") || inputString.startsWith("I")) {
        float nuevo = inputString.substring(1).toFloat();
        if (nuevo >= 0) {
          Ki = nuevo;
          Serial.print("-> Nuevo Ki = ");
          Serial.println(Ki, 3);
        }

      } else if (inputString.startsWith("d") || inputString.startsWith("D")) {
        float nuevo = inputString.substring(1).toFloat();
        if (nuevo >= 0) {
          Kd = nuevo;
          Serial.print("-> Nuevo Kd = ");
          Serial.println(Kd, 3);
        }

      } else {
        float nuevoSetpoint = inputString.toFloat();
        if (nuevoSetpoint >= 0 && nuevoSetpoint <= ALTURA_MAXIMA) {
          setpoint = nuevoSetpoint;
          Serial.print("-> Nuevo Setpoint: ");
          Serial.print(setpoint);
          Serial.println(" cm <-");
        } else {
          Serial.println("Comando no reconocido o fuera de rango.");
        }
      }

      inputString = "";  // limpiar cadena después de procesar
    } else if (isPrintable(c)) {
      inputString += c;
    }
  }
}
