# ros2_manipulator

Manipulador planar RRR (3 GDL) en ROS 2 Lyrical: cinemática, simulación y planificación.

## Paquetes
- `arm_kinematics`: modelo Denavit-Hartenberg y cinemática directa, con pruebas unitarias.

## Documentación
- [Cinemática directa por Denavit-Hartenberg](docs/cinematica_directa_DH.md)

## Uso
~~~bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
python3 -m pytest src/ros2_manipulator/arm_kinematics/test/test_kinematics.py -v
~~~
