Para correr el paquete en modo mapeo:
ros2 launch puzzlebot_nav2_gz_garden hexagonal_world.launch.py mode:=map use_sim_time:=True

Para poder mapear hay que lanzar el nodo de teleop_twist_keyboard por separado luego de correr el launch file con el modo de mapeo:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Para guardar el mapa hay que correr:
ros2 run nav2_map_server map_saver_cli -f my_map

Para correr el paquete en modo navegacion:
ros2 launch puzzlebot_nav2_gz_garden hexagonal_world.launch.py mode:=nav use_sim_time:=True

Se pueden cargar diferentes mapas para el puzzlebot. Para usar el mapa hexagonal usar:
map_name:=hexagonal

Para usar el mapa del laberinto del puzzlebot, usar:
map_name:=puzzlebot

El comando completo para navegacion seria:
ros2 launch puzzlebot_nav2_gz_garden gazebo_world.launch.py mode:=nav use_sim_time:=True map_name:=hexagonal

Es importante que al momento de crear un nuevo mapa para el puzzlebot, el nombre que se le de con el map_saver sea puzzlebot_map.yaml para seguir la convencion ya establecida. 

Por otro lado, antes de ejecutar con un mapa diferente. Hay que ir al archivo puzzlebot.yaml y colocar el nuevo nombre del map file en la linea 292.

Si el mapa se encuentra en la carpeta map no hay que pasar ningun parametro adicional a la ejecucion. El launch file ya lo referencia internamente.  

Para lanzar rqt_reconfigure (edicion dinamica de parametros):
ros2 run rqt_reconfigure rqt_reconfigure