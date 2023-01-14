PARA LA EJECUCION DE GAZEBO Y MAQUINA DE ESTADOS
--------------------------------------------------

TERMINAL 1:
   
	export TURTLEBOT3_MODEL=burger
	roslaunch turtlebot3_gazebo turtlebot3_house.launch

TERMINAL 2:

	export TURTLEBOT3_MODEL=burger
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/mapa2.yaml
	
TERMINAL 3:

	python3 GoToMapPoint.py

TERMINAL 4:

	python3 robot_maquina_estados_gazebo_vagabundo_go_to_point_x_y.py
 
 PARA LA DETECCION DE COLORES 
 --------------------------------
 
---------------------------------------------------------------------------------
METODO 1: Si se quiere usar directamente la webcam para detectar colores.

TERMINAL 5:

    python3 ColorCapture.py
  
TERMINAL 6:

    python3 SendImageToTopic.py 
  
----------------------------------------------------------------------------------
MÉTODO 2: Si se quiere simular por terminal la deteccion de los colores, se puede publicar directamente en el siguiente topic 
	
	rostopic pub /detect/azul std_msgs/Int16 "data: 1" 
	rostopic pub /detect/azul std_msgs/Int16 "data: 0" 
  
  rostopic pub /detect/rojo std_msgs/Int16 "data: 1" 
	rostopic pub /detect/rojo std_msgs/Int16 "data: 0"
  
  rostopic pub /detect/verde std_msgs/Int16 "data: 1" 
	rostopic pub /detect/verde std_msgs/Int16 "data: 0" 
 
