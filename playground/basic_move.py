#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import time
import math
import sys

# Ros Node process:
def main():
    # Initialize ROS and a ROS node
    rclpy.init(args=sys.argv)
    node = StraightCtrl()
    node.initializeROSnode()

    # Infinite Loop:
    rclpy.spin(node)

    # Clean end
    node.destroy_node()
    rclpy.shutdown()

# Ros Node Class:
class StraightCtrl(Node):
    def __init__(self):
        super().__init__('basic_move')
        self._logger = self.get_logger()
        self._pubVelocity = None
        self._subToScan = None
        self._timForCtrl = None

        self._obs_left = False
        self._obs_right = False
        self._rotate_left = False
        self._rotate_right = False
        self._last_move = 0

        self._nbr_points_right = 0
        self._nbr_points_left = 0

        self._bumper_state = None
        self._is_reversing = False
        
        self._can_move = False

        # Vitesse actuelle
        self._current_linear_velocity = 0.0
        self._current_angular_velocity = 0.0

        # Paramètres de lissage et limites
        self._velocity_smoothing_factor = 0.08
        self._max_linear_speed = 0.6
        self._max_angular_speed = 1.8
        self._low_linear_speed = 0.2
        self._low_angular_speed = 0.9

        # Détection de blocage
        self._direction_changes = 0  # Compte les alternances gauche/droite
        self._last_direction = 0     # Dernière direction utilisée (-1 = gauche, 1 = droite)
        self._stuck_timer = 0        # Timer pour détecter un blocage
        self._stuck_threshold = 5   # Seuil pour considérer le robot bloqué (nombre d'alternances)
        self._stuck_recovery_timer = 0  # Timer pour la récupération après blocage
        
        self.command_topic = self.declare_parameter('command_topic', '/multi/cmd_nav')

    def initializeROSnode(self):
        # Get logger from the node:
        self._logger = self.get_logger()

        # Initialize scan callback:
        self._subToPoints = self.create_subscription(
            PointCloud, '/points',
            self.scan_callback, 10
        )
        
        
                
        # Initialize publisher:
        self._pubVelocity = self.create_publisher(
            Twist, self.command_topic.value, 10
        )

        # Initialize control callback:
        self._timForCtrl = self.create_timer(
            0.05, self.control_callback
        )
        
        # Makes the robot starting automatically in the simulation 
        if self.command_topic.value == 'cmd_vel':
            self._can_move = True

    def scan_callback(self, pointCloud):
        self._nbr_points_right = 0
        self._nbr_points_left = 0
        self._obs_left = False
        self._obs_right = False
        for point in pointCloud.points:
            if point.y > 0:
                self._obs_left = True 
                self._nbr_points_left += 1
            else:
                self._obs_right = True
                self._nbr_points_right += 1

    def control_callback(self):
        
        if self._logger is None:
            return  
                
        if self._can_move:
        
            
            target_linear_velocity = 0.0
            target_angular_velocity = 0.0

            if self._bumper_state is not None and not self._is_reversing:
                
                self._is_reversing = True
                target_linear_velocity = -0.2
                target_angular_velocity = 0.0
                self._logger.info(f"Bumper activé ({self._bumper_state}). Marche arrière.")

                
                velocity_msg = Twist()
                velocity_msg.linear.x = target_linear_velocity
                velocity_msg.angular.z = target_angular_velocity
                self._pubVelocity.publish(velocity_msg)

                
                time.sleep(2)

                
                self._bumper_state = None
                self._is_reversing = False
                return

           
            if self._obs_right and self._obs_left:
              
                if self._nbr_points_left > self._nbr_points_right:
                    target_linear_velocity = 0.0
                    target_angular_velocity = -self._low_angular_speed
                    new_direction = -1
                else:
                    target_linear_velocity = 0.0
                    target_angular_velocity = self._low_angular_speed
                    new_direction = 1

                
                if new_direction != self._last_direction:
                    self._direction_changes += 1
                    self._stuck_timer += 1
                    self._last_direction = new_direction
                else:
                    self._stuck_timer = 0  # Réinitialiser si le mouvement est constant

                # Si trop d'alternances en peu de temps, considérer un blocage
                if self._direction_changes > self._stuck_threshold:
                    self._logger.info("Blocage détecté : activation de la récupération.")
                    self._stuck_recovery_timer = 50  # Temps pour effectuer une action de récupération
                    self._direction_changes = 0

            else:
                # Réinitialiser les compteurs si le robot peut se déplacer librement
                self._direction_changes = 0
                self._stuck_timer = 0

                if self._obs_right:
                    target_linear_velocity = self._low_linear_speed
                    target_angular_velocity = self._low_angular_speed
                    self._last_direction = 1
                elif self._obs_left:
                    target_linear_velocity = self._low_linear_speed
                    target_angular_velocity = -self._low_angular_speed
                    self._last_direction = -1
                else:
                    target_linear_velocity = self._max_linear_speed
                    target_angular_velocity = 0.0
                    self._last_direction = 0

            if self._stuck_recovery_timer > 0:
                
                target_linear_velocity = -self._low_linear_speed  
                if self._last_direction != 0:  
                    target_angular_velocity = self._max_angular_speed * self._last_direction
                else:
                    target_angular_velocity = self._max_angular_speed  
            
            self._stuck_recovery_timer -= 1

            
            self._current_linear_velocity += self._velocity_smoothing_factor * (target_linear_velocity - self._current_linear_velocity)
            self._current_angular_velocity += self._velocity_smoothing_factor * (target_angular_velocity - self._current_angular_velocity)

            
            velocity_msg = Twist()
            velocity_msg.linear.x = self._current_linear_velocity
            velocity_msg.angular.z = self._current_angular_velocity

            
            self._pubVelocity.publish(velocity_msg)
        
    

# Go:
if __name__ == '__main__':
    main()
