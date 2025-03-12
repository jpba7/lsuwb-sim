#!/usr/bin/env python3

import json
import os
from datetime import datetime
import re

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

        # Dicionário para rastrear inscrições ativas e seus últimos dados
        self.active_subscriptions = {}
        self.last_data = {}

        # Timer para verificar novos tópicos (a cada 5 segundos)
        self.topic_check_interval = 5.0
        rospy.Timer(rospy.Duration(self.topic_check_interval), self.check_topics)

        # Timer para envio em lote
        rospy.Timer(rospy.Duration(self.batch_interval), self.timer_callback)

        # Configuração da taxa de amostragem (Hz)
        self.sample_rate = float(os.getenv('SAMPLE_RATE', '10.0'))  # 10 Hz por padrão
        self.last_sample_time = {}

        rospy.loginfo("UWB API Bridge iniciado")

    def check_topics(self, event):
        """Verifica periodicamente por novos tópicos de tags UWB"""
        topics = rospy.get_published_topics()

        # Filtra tópicos UWB, excluindo os de PQF
        uwb_topics = []
        for topic, type_name in topics:
            if topic.startswith('/lsuwb/') and type_name == 'geometry_msgs/PoseStamped':
                if not topic.endswith('/pqf'):  # Ignora tópicos PQF
                    uwb_topics.append(topic)

        # Inscreve em novos tópicos
        for topic in uwb_topics:
            if topic not in self.active_subscriptions:
                tag_id = topic.split('/')[-1]  # Extrai o ID da tag do tópico
                rospy.loginfo(f"Nova tag detectada: {tag_id}")
                sub = rospy.Subscriber(
                    topic,
                    PoseStamped,
                    self.pose_callback,
                    callback_args=tag_id,
                    queue_size=1
                )
                self.active_subscriptions[topic] = sub
                self.last_sample_time[tag_id] = 0.0

        # Remove inscrições de tópicos que não existem mais e envia últimos dados
        for topic in list(self.active_subscriptions.keys()):
            if topic not in uwb_topics:
                tag_id = topic.split('/')[-1]
                rospy.loginfo(f"Removendo inscrição para tag ausente: {topic}")
                self.active_subscriptions[topic].unregister()
                del self.active_subscriptions[topic]

                # Envia últimos dados coletados da tag
                if tag_id in self.last_data:
                    last_position = self.last_data[tag_id]
                    self.batch_data.append(last_position)
                    del self.last_data[tag_id]

    def pose_callback(self, msg, tag_id):
        """Callback para processar mensagens de pose com throttling"""
        current_time = rospy.get_time()
        time_since_last_sample = current_time - self.last_sample_time.get(tag_id, 0.0)

        # Verifica se passou tempo suficiente desde a última amostra
        if time_since_last_sample >= (1.0 / self.sample_rate):
            position_data = {
                'tag_id': tag_id,
                'timestamp': msg.header.stamp.to_sec(),
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'created_at': datetime.now().isoformat()
            }
            self.batch_data.append(position_data)
            self.last_data[tag_id] = position_data
            self.last_sample_time[tag_id] = current_time
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
