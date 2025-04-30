Para correr el paquete en modo mapeo:
ros2 launch puzzlebot_nav2_gz_garden hexagonal_world.launch.py mode:=map use_sim_time:=True

Para poder mapear hay que lanzar el nodo de teleop_twist_keyboard por separado luego de correr el launch file con el modo de mapeo:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Para correr el paquete en modo navegacion:
ros2 launch puzzlebot_nav2_gz_garden hexagonal_world.launch.py mode:=nav use_sim_time:=True

Si el mapa se encuentra en la carpeta map no hay que pasar ningun parametro adicional a la ejecucion. El launch file ya lo referencia internamente.  