#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import Int32  # Importa mensagem para o pqf
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tag import Tag as TagClass

def main():
    rospy.init_node('lsuwb_node', anonymous=True)

    # Obtendo parâmetros do ROS
    serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
    baudrate = rospy.get_param("~baudrate", 115200)

    rospy.loginfo(f"Iniciando LSUWB com serial_port={serial_port} e baudrate={baudrate}")

    try:
        ser = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
    except serial.SerialException as e:
        rospy.logerr(f"Erro ao abrir porta serial {serial_port}: {e}")
        return

    publishers_pose = {}  # Dicionário para publicadores de PoseStamped
    publishers_pqf = {}   # Dicionário para publicadores de pqf

    try:
        # Inicialização do dispositivo
        rospy.loginfo("Enviando 0x0D duas vezes...")
        ser.write(b'\x0D')
        rospy.sleep(0.1)
        ser.write(b'\x0D')

        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                rospy.loginfo(f"Recebido: {data}")
                if "dwm>" in data:
                    rospy.loginfo("Comando 'lec' enviado!")
                    ser.write(b'lec\n')
                    break

        # Processamento das strings CSV recebidas
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                linha = ser.readline().decode('utf-8').strip()
                rospy.loginfo(f"Processando linha: {linha}")

                # Parse da linha CSV
                campos = linha.split(',')
                if campos[0] == "POS":
                    canal = int(campos[1])
                    ID = campos[2]
                    x = float(campos[3]) if campos[3] != 'nan' else None
                    y = float(campos[4]) if campos[4] != 'nan' else None
                    z = float(campos[5]) if campos[5] != 'nan' else None
                    pqf = int(campos[6], 16) if campos[6].startswith('x') else int(campos[6])

                    # Criar publishers se ainda não existirem
                    if ID not in publishers_pose:
                        publishers_pose[ID] = rospy.Publisher(f'/lsuwb/{ID}', PoseStamped, queue_size=10)
                        publishers_pqf[ID] = rospy.Publisher(f'/lsuwb/{ID}/pqf', Int32, queue_size=10)

                    # Publicar PoseStamped
                    msg_pose = PoseStamped()
                    msg_pose.header.stamp = rospy.Time.now()
                    msg_pose.header.frame_id = "map"
                    msg_pose.pose.position = Point(x or 0, y or 0, z or 0)
                    msg_pose.pose.orientation = Quaternion(0, 0, 0, 1)
                    
                    publishers_pose[ID].publish(msg_pose)

                    # Publicar pqf no novo tópico
                    msg_pqf = Int32()
                    msg_pqf.data = pqf
                    publishers_pqf[ID].publish(msg_pqf)

    except rospy.ROSInterruptException:
        rospy.loginfo("Encerrando a leitura.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
