PARA LA EJECUCION DE GAZEBO Y MAQUINA DE ESTADOS
--------------------------------------------------

TERMINAL 1:
   
	export TURTLEBOT3_MODEL=burger
	roslaunch turtlebot3_gazebo turtlebot3_house.launch

TERMINAL 2:
No olvidar pegar los archivos del mapa en la raiz del sistema 

	export TURTLEBOT3_MODEL=burger
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/mapa2.yaml
	
TERMINAL 3:

	python3 robot_vagabundo_maquina_estados_gazebo.py

 
 PARA LA DETECCION DE COLORES 
 --------------------------------
 
---------------------------------------------------------------------------------
METODO 1: Si se quiere usar directamente la webcam para detectar colores.

TERMINAL 5:

    python3 ColorCapture.py
  
TERMINAL 6:

    python3 SendImageToTopic.py 
  
----------------------------------------------------------------------------------
MÉTODO 2: Si se quiere simular por terminal la deteccion de los colores, se puede publicar directamente en el siguiente topic. Si quiero simular que se ha detectado el color publico un 1, mientras que si quiero simular que se ha dejado de detectar publico un 0. Si después de publicar un 1 no publico un 0 el topic se quedara activado a 1 por lo que se simulara que sigue detectando un color. 
	
	rostopic pub /detect/azul std_msgs/Int16 "data: 1" 
	rostopic pub /detect/azul std_msgs/Int16 "data: 0" 
  
	rostopic pub /detect/rojo std_msgs/Int16 "data: 1" 
	rostopic pub /detect/rojo std_msgs/Int16 "data: 0"
  
 	rostopic pub /detect/verde std_msgs/Int16 "data: 1" 
	rostopic pub /detect/verde std_msgs/Int16 "data: 0" 
 
