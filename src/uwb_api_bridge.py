#!/usr/bin/env python3

import json
import os
from datetime import datetime

from dotenv import load_dotenv
import requests
import rospy
from geometry_msgs.msg import PoseStamped

# Carrega variáveis de ambiente do arquivo .env
load_dotenv()


class UWBApiBridge:
    def __init__(self):
        # Configurações da API
        self.api_url = os.getenv('API_URL', 'http://localhost:8000/devices/api/ros/device-datapoints/')
        self.api_token = os.getenv('API_TOKEN', '')
        self.batch_interval = float(os.getenv('BATCH_INTERVAL', '60'))  # 5 minutos
        self.local_data_file = '/tmp/uwb_pending_data.json'

        # Configuração dos headers da API
        self.headers = {
            'Content-Type': 'application/json',
            # 'Authorization': f'Bearer {self.api_token}'
        }

        # Buffer para armazenar dados
        self.batch_data = []

        # Inicializa nó ROS
        rospy.init_node('uwb_api_bridge', anonymous=True)

        # Subscribe em todos os tópicos de posição
        for tag_id in ['Tag1', 'Tag2', 'Tag3']:
            rospy.Subscriber(
                f'/lsuwb/{tag_id}',
                PoseStamped,
                self.pose_callback,
                callback_args=tag_id
            )

        # Timer para envio em lote
        rospy.Timer(rospy.Duration(self.batch_interval), self.timer_callback)

        rospy.loginfo("UWB API Bridge iniciado")

    def pose_callback(self, msg, tag_id):
        """Callback para processar mensagens de pose"""
        position_data = {
            'tag_id': tag_id,
            'timestamp': msg.header.stamp.to_sec(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'created_at': datetime.now().isoformat()
        }
        self.batch_data.append(position_data)
        rospy.logdebug(f"Dados recebidos da {tag_id}: {position_data}")

    def load_pending_data(self):
        """Carrega dados pendentes do arquivo local"""
        if not os.path.exists(self.local_data_file):
            return []

        try:
            with open(self.local_data_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f'Erro ao carregar dados pendentes: {e}')
            return []

    def save_pending_data(self, data):
        """Salva dados pendentes no arquivo local"""
        try:
            with open(self.local_data_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            rospy.logerr(f'Erro ao salvar dados pendentes: {e}')

    def send_to_api(self, data_batch):
        """Envia dados para a API"""
        if not data_batch:
            return True

        try:
            response = requests.post(
                self.api_url,
                headers=self.headers,
                json={'positions': data_batch},
                timeout=10
            )

            if response.status_code in [200, 201]:
                rospy.loginfo(f'Enviados {len(data_batch)} pontos com sucesso')
                return True
            else:
                rospy.logwarn(f'Falha no envio. Status={response.status_code}')
                return False

        except Exception as e:
            rospy.logerr(f'Erro na requisição: {e}')
            return False

    def timer_callback(self, event):
        """Processa envio em lote periodicamente"""
        # Processa dados pendentes primeiro
        pending_data = self.load_pending_data()
        if pending_data:
            if self.send_to_api(pending_data):
                self.save_pending_data([])

        # Processa batch atual
        if self.batch_data:
            current_batch = list(self.batch_data)
            self.batch_data = []

            if not self.send_to_api(current_batch):
                # Em caso de falha, salva localmente
                all_pending = self.load_pending_data()
                all_pending.extend(current_batch)
                self.save_pending_data(all_pending)

    def run(self):
        """Executa o bridge"""
        rospy.spin()


if __name__ == '__main__':
    try:
        bridge = UWBApiBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
