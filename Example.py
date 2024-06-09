import AR4_api
import time


robot = AR4_api.AR4("COM9")
robot.open()
time.sleep(1)
robot.cal_robot_all()

robot.move_j(362.295, 148.723, 152.148, 179.997, 0.058, 179.990, speed=40)
robot.move_l(362.347, 148.746, 72.901, 179.981, 0.091, 179.968, speed=25)
time.sleep(1)
robot.move_l(362.366, 148.751, 157.191, 179.983, 0.094, 179.990, speed=25)
robot.move_j(398.521, 147.901, 192.077, -154.794, 87.618, -156.102, speed=40, wrist_config='N')
robot.move_l(398.486, -159.466, 192.096, -154.749, 87.652, -156.052, speed=40, wrist_config='N')
robot.move_j(293.975, -158.974, 157.285, -179.523, 1.119, -179.158, speed=40)
robot.move_l(294.049, -159.015, 75.139, -179.505, 1.148, -179.157, speed=25)
time.sleep(1)
robot.move_l(294.071, -159.029, 158.539, -179.504, 1.144, -179.138, speed=25)
robot.move_j(453.084, 0.201, 158.609, -179.454, 1.245, -179.055, speed=40)
robot.move_j(20.025, 0.255, 666.752, -0.389, 1.309, -0.918, speed=40, wrist_config='N')
robot.move_j(132.288, 0.167, 512.871, -2.670, 67.958, -2.794, speed=40, wrist_config='N')

robot.close()