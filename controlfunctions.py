import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import pygame

# control topic for velocity and steering angle
from autoware_auto_control_msgs.msg import AckermannControlCommand

# planning topic for setting and clearing speed limit
from tier4_planning_msgs.msg import VelocityLimit, VelocityLimitClearCommand


def get_keys():
    keys = pygame.key.get_pressed()
    return keys
    
def switch_control(keys):
    global controlToggle
    if keys[pygame.K_i]:
        controlToggle = not controlToggle
        if controlToggle:
            print("Manual mode enabled ")
        else:
            print("Manual mode disabled")

def get_input(speed, keys):
    msg = AckermannControlCommand()
    if keys[pygame.K_e]:
        msg.longitudinal.speed = 0.0
        msg.longitudinal.acceleration = -200.0
        msg.lateral.steering_tire_angle = 0.0
    if keys[pygame.K_w]:
        msg.longitudinal.acceleration = 5.0
    if keys[pygame.K_a]:
        msg.lateral.steering_tire_angle = 0.1
    if keys[pygame.K_s]:
        msg.longitudinal.acceleration = -5.0
    if keys[pygame.K_d]:
        msg.lateral.steering_tire_angle = -0.1
    if keys[pygame.K_w] and keys[pygame.K_s]:
        msg.longitudinal.acceleration = 0.0
    if keys[pygame.K_a] and keys[pygame.K_d]:
        msg.longitudinal.acceleration = 0.0
    msg.longitudinal.speed = speed
    return msg

# 
def set_speed_limit(msg, speed_limit=13.89):
    msg.max_velocity = speed_limit
    return msg