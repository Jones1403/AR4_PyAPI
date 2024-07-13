# AR4 Robot Python API

Welcome to the AR4 Robot Python API! This library provides an easy-to-use interface for controlling the AR4 robot. With this API, you can command the robot to perform various movements such as joint movements (MoveJ), linear movements (MoveL), arc movements (MoveA), and circular movements (MoveC).

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Table of Contents
- [Getting Started](#getting-started)
- [API Usage](#api-usage)
  - [Calibration Command](#calibration-command)
  - [Joint Calibration Command](#joint-calibration-command)
  - [MoveR Command](#mover-command)
  - [MoveJ Command](#movej-command)
  - [MoveL Command](#movel-command)
  - [MoveA Command](#movea-command)
  - [MoveC Command](#movec-command)
  - [Other Commands] (#other-commands)
- [Contributing](#contributing)
- [License](#license)

## Getting Started

To use the AR4 API, you first need to import the library and create a robot object by specifying the appropriate COM port. Here is a simple example to get you started:

```python
import AR4_api

# Replace 'COMx' with the appropriate COM port for your robot and 'COMy' with the  COM port for the pneumatic or servo gripper.
robot = AR4_api.AR4("COMx")
robot.open()
robot.set_com_gripper("COMy")
```
While opening the robot, the API imports the calibration file that is generated by the GUI application provided by Annin Robotics. This file must be available in the sources root.

## API Usage

### Calibration Command

The `cal_robot_all` command is used to calibrate or home the robot as it is done in the GUI application provided by Annin Robotics. In the first stage joints 1, 2 and 4 are calibrated, in the second stage joints 3, 5 and 6 are calibrated.

```python
# Example: calibrate all joints of the robot with two stages.
robot.cal_robot_all()
```

### Joint Calibration Command

The `cal_robot_joint` command is used to calibrate or home the a specific joint on the robot.

```python
# Example: calibrate a joint of the robot.
robot.cal_robot_joint(1)
# calibrates joint 1 on the robot.
```

### MoveR Command

The `MoveR` command is used for joint movements. This command moves the joints of the robot to a specific angle

```python
# Example: Move joints to a specific position
move_r(j1, j2, j3, j4, j5, j6, j7=0.0, j8=0.0, j9=0.0, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F')
```

### MoveJ Command

The `MoveJ` command is used for a joint movement. This command moves the robot to a specified position in carthesian coordinates

```python
# Example: Move to a specific joint position
robot.move_j(x, y, z, rx, ry, rz, j7=0.0, j8=0.0, j9=0.0, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F'):
```

### MoveL Command

The `MoveL` command is used for linear movements. This command moves the robot's end-effector in a straight line to the specified position.

```python
# Example: Move to a specific Cartesian position in a linear motion
robot.move_l(x, y, z, rx, ry, rz, j7=0.0, j8=0.0, j9=0.0, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, rnd=0, wrist_config='F', dis_wrist=False)
```

### MoveA Command

The `move_a` command is used for arc movements. This command moves the robot's end-effector along an arc from the current position defined by a point on the arc and the end point.

```python
# Example: Move along an arc. First move_j or move_l to the start position.
robot.move_l(x, y, z, rx, ry, rz, j7=0.0, j8=0.0, j9=0.0, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, rnd=0, wrist_config='F', dis_wrist=False)
#next start the arc move by defining the mid and end positions of the robot along the arc.
move_a(x, y, z, rx, ry, rz, x_end, y_end, z_end, tr_val, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F')
```

### MoveC Command

The `move_c` command is used for circular movements. This command moves the robot's end-effector along a circular path defined by two intermediate points and an end point.

```python
# Example: Move along a circular path
move_c(x_center, y_center, z_center, rx, ry, rz, x_start, y_start, z_start, x_plain, y_plain, z_plain, tr_val, spd_prefix='Sp', speed=25, acceleration=20, deceleration=20, acc_ramp=100, wrist_config='F')
```

### Other Commands
Apart from these move and calibration commands, the API provides a series of other commands including:
```
set_io_arduino(output, state) 	# to set an output on the arduino nano in the pneumatic gripper assembly
set_io_teensy(output, state)	# to set an output on the teensy
start_spline()
end_spline(stop_queue=False)
test_limit_switches()			#returns the response of the robot which includes the state of the limit switches
set_encoders()					#sets the encoders to a value of 1000
read_encoders()					#returns the current position of the encoders
set_tcp(self, x, y, z, rx, ry, rz)	`#sets the tool center point.
servo_cmd(number, position)		#sets the servo gripper to a specific location
```



## Contributing

We welcome contributions to improve the AR4 API. If you have any suggestions or find any issues, please open an issue or submit a pull request on the [GitHub repository](https://github.com/Jones1403/AR4_PyAPI).

## License

This project is licensed under the GPL V3.0 License. See the [LICENSE](LICENSE) file for more details.

---

Thank you for using the AR4 Robot Python API! If you have any questions or need further assistance, please feel free to reach out.
