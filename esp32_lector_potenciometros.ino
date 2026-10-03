/*
  ESP32 - Lector de potenciómetros para control de brazo robótico (URDF: robot_con_pinza)
  ------------------------------------------------------------------------------------
  Cada potenciómetro representa una articulación del brazo:
    POT1 -> joint_1        (revolute,  -2.5 a  2.5 rad)  base
    POT2 -> joint_2        (revolute,  -2.0 a  2.0 rad)  codo
    POT3 -> joint_gripper  (prismatic,  0.0 a  0.15 m)   extensión de la pinza
    POT4 -> pinza (abre/cierra los 2 dedos en espejo, 0.0 a 0.05 m)
             -> joint_dedo_izq y joint_dedo_der

  Conexión de cada potenciómetro:
    Terminal izq -> 3V3
    Terminal der -> GND
    Cursor (wiper) -> pin ADC indicado abajo

  IMPORTANTE: usar pines ADC1 (32-39) para no tener conflicto si luego usas WiFi/BT.

  Formato enviado por Serial (1 línea cada ~50 ms), fácil de parsear en Python:
    J1:<rad>,J2:<rad>,GRIP:<m>,PINZA:<m>\n

  Ejemplo real:
    J1:0.842,J2:-1.203,GRIP:0.071,PINZA:0.032
*/

const int PIN_POT_J1    = 34;
const int PIN_POT_J2    = 35;
const int PIN_POT_GRIP  = 32;
const int PIN_POT_PINZA = 33;

const int ADC_MIN = 0;
const int ADC_MAX = 4095;   // ESP32 ADC de 12 bits

// Límites tomados directamente del URDF (elemento <limit>)
const float J1_MIN = -2.5, J1_MAX = 2.5;
const float J2_MIN = -2.0, J2_MAX = 2.0;
const float GRIP_MIN = 0.0, GRIP_MAX = 0.15;
const float PINZA_MIN = 0.0, PINZA_MAX = 0.05;

const unsigned long INTERVALO_MS = 50;   // 20 Hz
unsigned long ultimoEnvio = 0;

// Filtro exponencial simple para suavizar el ruido del ADC
float filtroJ1 = 0, filtroJ2 = 0, filtroGrip = 0, filtroPinza = 0;
const float ALPHA = 0.2;  // 0 = muy suave/lento, 1 = sin filtro

float mapFloat(long x, long in_min, long in_max, float out_min, float out_max) {
  return out_min + (out_max - out_min) * ((float)(x - in_min) / (float)(in_max - in_min));
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);      // 0-4095
  analogSetAttenuation(ADC_11db); // rango completo 0-3.3V

  // Toma de valores iniciales para que el filtro no arranque en 0
  filtroJ1    = analogRead(PIN_POT_J1);
  filtroJ2    = analogRead(PIN_POT_J2);
  filtroGrip  = analogRead(PIN_POT_GRIP);
  filtroPinza = analogRead(PIN_POT_PINZA);
}

void loop() {
  int lecturaJ1    = analogRead(PIN_POT_J1);
  int lecturaJ2    = analogRead(PIN_POT_J2);
  int lecturaGrip  = analogRead(PIN_POT_GRIP);
  int lecturaPinza = analogRead(PIN_POT_PINZA);

  // Filtro exponencial (suaviza el "salto" típico de potenciómetros baratos)
  filtroJ1    += ALPHA * (lecturaJ1    - filtroJ1);
  filtroJ2    += ALPHA * (lecturaJ2    - filtroJ2);
  filtroGrip  += ALPHA * (lecturaGrip  - filtroGrip);
  filtroPinza += ALPHA * (lecturaPinza - filtroPinza);

  if (millis() - ultimoEnvio >= INTERVALO_MS) {
    ultimoEnvio = millis();

    float j1    = mapFloat((long)filtroJ1,    ADC_MIN, ADC_MAX, J1_MIN,    J1_MAX);
    float j2    = mapFloat((long)filtroJ2,    ADC_MIN, ADC_MAX, J2_MIN,    J2_MAX);
    float grip  = mapFloat((long)filtroGrip,  ADC_MIN, ADC_MAX, GRIP_MIN,  GRIP_MAX);
    float pinza = mapFloat((long)filtroPinza, ADC_MIN, ADC_MAX, PINZA_MIN, PINZA_MAX);

    Serial.print("J1:");    Serial.print(j1, 3);
    Serial.print(",J2:");   Serial.print(j2, 3);
    Serial.print(",GRIP:"); Serial.print(grip, 3);
    Serial.print(",PINZA:");Serial.println(pinza, 3);
  }
}
