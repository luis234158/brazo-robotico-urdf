"""
Control de brazo robótico (robot_con_pinza.urdf) desde datos UART del ESP32.
-----------------------------------------------------------------------------
Requisitos:
    pip install pybullet pyserial

Uso normal (con el ESP32 conectado):
    python control_brazo.py --port COM5          # Windows
    python control_brazo.py --port /dev/ttyUSB0  # Linux/Mac

Modo de prueba SIN hardware (genera movimiento de ejemplo, útil para probar
la simulación mientras armas el circuito o para las capturas de pantalla):
    python control_brazo.py --sim

El ESP32 debe enviar líneas de texto por Serial con este formato:
    J1:<rad>,J2:<rad>,GRIP:<m>,PINZA:<m>
Ejemplo:
    J1:0.842,J2:-1.203,GRIP:0.071,PINZA:0.032
"""

import argparse
import re
import sys
import time

import pybullet as p
import pybullet_data

# Límites de cada articulación, tomados del URDF (para "clampear" por seguridad)
LIMITS = {
    "joint_1":       (-2.5, 2.5),
    "joint_2":       (-2.0, 2.0),
    "joint_gripper": (0.0, 0.15),
    "joint_dedo_izq":(0.0, 0.05),
    "joint_dedo_der":(0.0, 0.05),
}

LINE_RE = re.compile(
    r"J1:(-?\d+\.?\d*),J2:(-?\d+\.?\d*),GRIP:(-?\d+\.?\d*),PINZA:(-?\d+\.?\d*)"
)


def clamp(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


def construir_indices_joints(robot_id):
    """Mapea nombre_de_joint -> índice interno de PyBullet."""
    nombre_a_indice = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        nombre = info[1].decode("utf-8")
        nombre_a_indice[nombre] = i
    return nombre_a_indice


def mover_joints(robot_id, nombre_a_indice, j1, j2, grip, pinza):
    j1 = clamp(j1, *LIMITS["joint_1"])
    j2 = clamp(j2, *LIMITS["joint_2"])
    grip = clamp(grip, *LIMITS["joint_gripper"])
    pinza = clamp(pinza, *LIMITS["joint_dedo_izq"])  # mismo rango en ambos dedos

    objetivo = {
        "joint_1": j1,
        "joint_2": j2,
        "joint_gripper": grip,
        "joint_dedo_izq": pinza,
        "joint_dedo_der": pinza,  # los dos dedos se mueven en espejo con el mismo valor
    }

    for nombre, valor in objetivo.items():
        idx = nombre_a_indice[nombre]
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=valor,
            force=200,
        )


def leer_desde_serial(puerto, baudrate=115200):
    import serial  # pyserial

    ser = serial.Serial(puerto, baudrate, timeout=1)
    time.sleep(2)  # esperar a que el ESP32 reinicie tras abrir el puerto
    ser.reset_input_buffer()

    while True:
        try:
            linea = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception as e:
            print(f"Error leyendo serial: {e}", file=sys.stderr)
            continue

        if not linea:
            continue

        match = LINE_RE.search(linea)
        if match:
            j1, j2, grip, pinza = (float(v) for v in match.groups())
            yield j1, j2, grip, pinza
        else:
            # Línea con ruido/basura, o de arranque del ESP32: se ignora
            print(f"[ignorado] {linea}")


def generar_datos_simulados():
    """Genera movimiento senoidal de ejemplo, sin necesidad del ESP32."""
    t0 = time.time()
    while True:
        t = time.time() - t0
        j1 = 2.0 * (0.5 - abs((t * 0.3) % 2 - 1))       # triángulo entre -1 y 1 aprox
        j2 = 1.5 * (0.5 - abs((t * 0.2) % 2 - 1))
        grip = 0.075 + 0.075 * ((t * 0.5) % 2 - 1) * 0.5
        pinza = 0.025 + 0.025 * ((t * 0.8) % 2 - 1) * 0.5
        yield j1, j2, max(0.0, grip), max(0.0, pinza)
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="Control del brazo URDF vía UART/ESP32")
    parser.add_argument("--port", help="Puerto serial del ESP32, p. ej. COM5 o /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--urdf", default="robot_con_pinza.urdf")
    parser.add_argument("--sim", action="store_true",
                         help="Modo de prueba sin ESP32: genera datos de ejemplo")
    args = parser.parse_args()

    if not args.sim and not args.port:
        parser.error("Debes indicar --port (ESP32) o usar --sim para probar sin hardware")

    # --- Inicializar PyBullet ---
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(args.urdf, basePosition=[0, 0, 0], useFixedBase=True)
    nombre_a_indice = construir_indices_joints(robot_id)
    print("Articulaciones encontradas en el URDF:", list(nombre_a_indice.keys()))

    fuente_datos = generar_datos_simulados() if args.sim else leer_desde_serial(args.port, args.baud)
    if args.sim:
        print(">> Modo SIMULACIÓN (sin ESP32): moviendo el brazo con datos de ejemplo")
    else:
        print(f">> Escuchando ESP32 en {args.port} @ {args.baud} baudios")

    try:
        for j1, j2, grip, pinza in fuente_datos:
            mover_joints(robot_id, nombre_a_indice, j1, j2, grip, pinza)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)  # frecuencia típica de simulación de PyBullet
    except KeyboardInterrupt:
        print("\nCerrando...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
