# Brazo Robótico URDF — ESP32 + Python (PyBullet)

Proyecto U_Militar: control de un brazo robótico (definido en `robot_con_pinza.urdf`) usando 4 potenciómetros conectados a un ESP32, comunicados por UART con un script en Python que mueve el brazo en una simulación 3D con PyBullet.

## Archivos

- **`robot_con_pinza.urdf`** — Descripción del brazo: 5 articulaciones (`joint_1`, `joint_2`, `joint_gripper`, `joint_dedo_izq`, `joint_dedo_der`).
- **`esp32_lector_potenciometros.ino`** — Firmware del ESP32. Lee 4 potenciómetros, los convierte a los rangos reales de cada articulación (según los límites del URDF) y los envía por Serial a 115200 baudios.
- **`control_brazo.py`** — Script en Python. Recibe los datos por UART (o genera datos de ejemplo con `--sim`) y mueve el brazo simulado en PyBullet en tiempo real.

## Cómo funciona

1. Cada potenciómetro representa una articulación:
   - POT1 → `joint_1` (base, gira)
   - POT2 → `joint_2` (codo, gira)
   - POT3 → `joint_gripper` (extensión de la pinza)
   - POT4 → apertura/cierre de la pinza (controla `joint_dedo_izq` y `joint_dedo_der` en espejo)
2. El ESP32 lee los 4 potenciómetros, los filtra (suavizado) y los envía por USB como texto:
   `J1:0.842,J2:-1.203,GRIP:0.071,PINZA:0.032`
3. El script de Python lee esa línea, la convierte en los 4 valores, y le dice a PyBullet que mueva cada articulación a esa posición (`POSITION_CONTROL`), dentro de los límites de seguridad definidos en el URDF.

## Requisitos

```
pip install pybullet pyserial
```

(En Windows, `pybullet` puede requerir compilarse; se recomienda usar un entorno con Python 3.11 vía conda/Miniforge e instalar con `conda install -c conda-forge pybullet -y` para evitar necesitar Visual C++ Build Tools.)

## Uso

Con el ESP32 conectado (sube antes `esp32_lector_potenciometros.ino` desde el Arduino IDE):

```
python control_brazo.py --port COM5
```

Modo de prueba sin hardware (genera movimiento de ejemplo):

```
python control_brazo.py --sim
```

## Autor

Luis Miguel Ruiz — Ingeniería Mecatrónica, Universidad Militar Nueva Granada.
