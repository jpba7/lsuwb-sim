#!/usr/bin/env python3

import serial
import time
import rospy
from lpuwb.msg import Tag
from src.tag import Tag as TagClass

# Dicionário para gerenciar as instâncias de Tag
tags = {}

# Configuração da porta serial
ser = serial.Serial(
    port='/dev/ttyACM0',         # Substitua pela porta do seu dispositivo
    baudrate=115200,            # Taxa de transmissão (ajuste conforme necessário)
    timeout=1                   # Tempo de espera para leitura (em segundos)
)

def publicar_tag(tag_obj, publisher):
    """
    Publica os dados de uma Tag no tópico correspondente.
    """
    msg = Tag()
    msg.canal = tag_obj.canal
    msg.ID = tag_obj.ID
    msg.x = tag_obj.x if tag_obj.x is not None else float('nan')
    msg.y = tag_obj.y if tag_obj.y is not None else float('nan')
    msg.z = tag_obj.z if tag_obj.z is not None else float('nan')
    msg.pqf = tag_obj.pqf
    publisher.publish(msg)

def main():
    rospy.init_node('lpuwb_node', anonymous=True)
    publishers = {}  # Dicionário para gerenciar publicadores de cada Tag
    rospy.loginfo("Nodo lpuwb iniciado!")

    try:
        # Inicialização (enviar os 0x0D e esperar "dwm>")
        rospy.loginfo("Enviando 0x0D duas vezes...")
        ser.write(b'\x0D')
        time.sleep(0.1)
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
                if campos[0] == "POS":  # Verifica se é uma linha válida de posição
                    canal = int(campos[1])
                    ID = campos[2]
                    x = float(campos[3]) if campos[3] != 'nan' else None
                    y = float(campos[4]) if campos[4] != 'nan' else None
                    z = float(campos[5]) if campos[5] != 'nan' else None
                    pqf = int(campos[6], 16) if campos[6].startswith('x') else int(campos[6])

                    # Atualiza ou cria a Tag no dicionário
                    if ID not in tags:
                        tags[ID] = TagClass(canal, ID, x, y, z, pqf)
                        publishers[ID] = rospy.Publisher(f'/tag/{ID}', Tag, queue_size=10)
                        rospy.loginfo(f"Nova Tag criada: {tags[ID]}")
                    else:
                        tags[ID].atualizar_dados(x, y, z, pqf)

                    # Publica os dados da Tag
                    publicar_tag(tags[ID], publishers[ID])

    except rospy.ROSInterruptException:
        rospy.loginfo("Encerrando a leitura.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
