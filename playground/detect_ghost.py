#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from kobuki_ros_interfaces.msg import Sound

# Classe principale du nœud ROS
class GreenObjectDetector:
    def __init__(self):
        self._logger = None
        self._publisher = None
        self._subscriber = None
        self._bridge = CvBridge()
        self._ghost_detected = False
        self._detection_threshold = 4000

    def initializeROSnode(self, ros_node):
        # Initialiser le logger
        self._logger = ros_node.get_logger()

        # Initialiser le publisher
        self._publisher = ros_node.create_publisher(
            String, 'green_object_detected', 10
        )
        
        self._sound_publisher = ros_node.create_publisher(
            Sound, 'commands/sound', 10
        )

        # Initialiser le subscriber
        self._subscriber = ros_node.create_subscription(
            Image, 'sensor_msgs/image', self.image_callback, 10
        )

    def image_callback(self, msg):
        # Convertir le message ROS en image OpenCV
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convertir l'image de l'espace BGR à l'espace HSV
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Définir la plage de couleur pour détecter le vert (en HSV)
        lower_green = np.array([35, 100, 50])  # Ajuster selon vos besoins
        upper_green = np.array([85, 255, 255])

        # Créer un masque basé sur la plage de couleur verte
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Compter le nombre de pixels verts dans le masque
        green_pixel_count = cv2.countNonZero(mask)

        # self._logger.info(f"{green_pixel_count} pixels verts détéctés.")

        # Vérifier si le nombre de pixels verts dépasse un certain seuil
        if green_pixel_count > self._detection_threshold and not self._ghost_detected:
            self._publisher.publish(String(data="Fantôme détecté!"))
            sound = Sound()
            sound.value = 4
            self._sound_publisher.publish(sound)
            
            self._ghost_detected = True

        if green_pixel_count < self._detection_threshold:
            self._ghost_detected = False

        # Afficher l'image originale et l'image masquée (pour le débogage)
        cv2.imshow('Image Originale', frame)
        cv2.imshow('Masque Vert', mask)
        cv2.waitKey(1)  # Attendre 1 ms pour permettre l'affichage


# Fonction principale du nœud ROS
def main():
    # Initialiser ROS et un nœud ROS
    rclpy.init()
    node = Node('green_object_detector')

    # Créer une instance de la classe de contrôle et initialiser le nœud
    green_object_detector = GreenObjectDetector()
    green_object_detector.initializeROSnode(node)

    # Boucle infinie pour exécuter le nœud
    rclpy.spin(node)

    # Arrêter proprement
    node.destroy_node()
    rclpy.shutdown()

# Déclencheur du script
if __name__ == '__main__':
    main()
