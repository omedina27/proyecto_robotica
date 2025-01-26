from pyniryo import *  # Librería para controlar el robot Niryo
import math
import serial  # Para la comunicación con el Arduino

# Configuración del robot Niryo
robot = NiryoRobot("10.10.10.10")

robot.calibrate_auto()

# Configuración del puerto Serial para la comunicación con el Arduino
try:
    arduino = serial.Serial(port="COM4", baudrate=9600, timeout=1)  # Ajusta el puerto según tu configuración
    print("Conexión establecida con el Arduino.")
except serial.SerialException as e:
    print(f"Error al conectar con el Arduino: {e}")
    robot.close_connection()  # Asegura cerrar la conexión con el robot antes de salir
    exit()

try:
    while True:
        # Leer la distancia desde el Arduino
        if arduino.in_waiting > 0:
            try:
                raw_data = arduino.readline().decode().strip()  # Leer y decodificar los datos
                print(f"Datos recibidos del Arduino: '{raw_data}'")  # Imprimir datos crudos para depuración

                # Convertir los datos a float (validar si son un número)
                distancia_cm = float(raw_data)
                distancia_m = distancia_cm / 100.0

                # Configuración de posiciones
                x_sensor = distancia_m  # Coordenada fija del sensor en x (m)
                y_fija = 0.0  # Coordenada fija del sensor en y (m)
                z_fija = 0.113
                # Altura fija del objeto en el marco del robot (m)

                pick_pose = PoseObject(
                    x=0.08 + x_sensor,
                    y=y_fija,
                    z=z_fija,  # Altura fija
                    roll=0,  # Orientación del efector final
                    pitch=1.57,  # Vertical hacia abajo
                    yaw=0.0,  # Sin rotación horizontal
                )

                # Validar distancia y realizar movimiento
                if 0.24 > pick_pose.x -0.08 > 0.09:  # Validar que la distancia sea válida
                    print(f"Moviendo a: x={x_sensor:.2f}, y={y_fija:.2f}, z={z_fija:.2f}")
                    
                    # Pick del objeto
                    robot.pick_from_pose(*pick_pose.to_list())

                    # Place del objeto
                    place_pose = pick_pose.copy_with_offsets(x_offset=-pick_pose.x, y_offset=0.2)
                    robot.place_from_pose(*place_pose.to_list())

                    # Resetear el buffer de entrada del puerto serial
                    arduino.reset_input_buffer()
                    print("Buffer serial limpiado. Listo para nueva lectura.")
                    robot.wait(2)
                else:
                    print("Distancia inválida, esperando lectura válida...")

            except ValueError:
                print("Error: no se pudo convertir los datos recibidos a un número. Reintentando...")

except KeyboardInterrupt:
    print("Conexión terminada por el usuario.")
finally:
    print("Cerrando conexiones...")
    robot.close_connection()  # Cerrar la conexión con el robot
    arduino.close()  # Cerrar la conexión con el Arduino