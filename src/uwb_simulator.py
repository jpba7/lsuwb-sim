#!/usr/bin/env python3

import random

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Int32


class UWBSimulator:
    def __init__(self):
        rospy.init_node('uwb_simulator', anonymous=True)

        # Configurações das TAGs
        self.tags_config = {
            'Tag1': {'x_range': (0, 200), 'y_range': (0, 100), 'z_range': (1, 2)},
            'Tag2': {'x_range': (30, 140), 'y_range': (20, 80), 'z_range': (0.7, 1.8)},
            'Tag3': {'x_range': (50, 100), 'y_range': (50, 100), 'z_range': (0, 10)}
        }

        # Dicionários para armazenar os publishers
        self.pose_publishers = {}
        self.pqf_publishers = {}

        # Criar publishers para cada TAG
        for tag_id in self.tags_config.keys():
            self.pose_publishers[tag_id] = rospy.Publisher(
                f'/lsuwb/{tag_id}',
                PoseStamped,
                queue_size=10
            )
            self.pqf_publishers[tag_id] = rospy.Publisher(
                f'/lsuwb/{tag_id}/pqf',
                Int32,
                queue_size=10
            )

        self.rate = rospy.Rate(0.2)  # 10 Hz
        rospy.loginfo("Simulador UWB iniciado")

    def generate_random_pose(self, config):
        """Gera uma posição aleatória dentro dos limites definidos"""
        x = random.uniform(*config['x_range'])
        y = random.uniform(*config['y_range'])
        z = random.uniform(*config['z_range'])
        return Point(x, y, z)

    def generate_random_pqf(self):
        """Gera um valor PQF aleatório entre 0 e 100"""
        return random.randint(50, 100)

    def create_pose_message(self, position):
        """Cria uma mensagem PoseStamped"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position = position
        msg.pose.orientation = Quaternion(0, 0, 0, 1)
        return msg

    def run(self):
        """Loop principal do simulador"""
        while not rospy.is_shutdown():
            for tag_id, config in self.tags_config.items():
                # Gerar e publicar pose
                position = self.generate_random_pose(config)
                pose_msg = self.create_pose_message(position)
                self.pose_publishers[tag_id].publish(pose_msg)

                # Gerar e publicar PQF
                pqf_msg = Int32(self.generate_random_pqf())
                self.pqf_publishers[tag_id].publish(pqf_msg)

                rospy.loginfo(f"TAG {tag_id}: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        simulator = UWBSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
