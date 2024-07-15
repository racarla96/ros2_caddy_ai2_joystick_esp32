# ros2_caddy_ai2_joystick_esp32


## Instalación de docker en Linux

Seguir los pasos del siguiente enlace: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

Asignar permisos al usuario actual
```bash
sudo usermod -aG docker $USER
```

###

```bash
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0
```

Nota: Ajuste /dev/ttyUSB0 según el puerto serie de su dispositivo. Para ver los dispositivos dispositivos recientemente conectados puedes usar:

```bash
sudo dmesg | grep tty
```

Si no puede cargar el código de Platformio.

```bash
sudo usermod -a -G dialout $USER
```

### Enlaces
- https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
- https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html
- https://docs.ros2.org/latest/api/std_msgs/msg/Header.html
- https://github.com/micro-ROS/micro_ros_platformio
- https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy
- https://github.com/micro-ROS/micro_ros_arduino
- https://github.com/micro-ROS/micro_ros_arduino/blob/jazzy/examples/micro-ros_tf_publisher/micro-ros_tf_publisher.ino
- https://github.com/micro-ROS/micro_ros_arduino/blob/jazzy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
- https://github.com/micro-ROS/micro-ROS-demos/blob/jazzy/rclc/string_publisher/main.c