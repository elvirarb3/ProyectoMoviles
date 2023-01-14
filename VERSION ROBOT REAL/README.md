ANTES DE EMPEZAR LA EJECUCIÓN
----------------------------------------------------------------
Este archivo define las variables de entorno, donde el número final es el referente a la ip del turtlebot

	bash setvars.sh 1
	
Importante tambien copiar los archivos del mapa dentro del turtlebot, en una terminal vamos hasta la carpeta de los archivos de los mapas y escribimos el siguiente comando

	scp mapa_eps.pgm mapa_eps.yaml turtlebot@192.168.1.5:~
	
PARA LA EJECUCIÓN EN EL TURTLEBOT Y MAQUINA DE ESTADOS
--------------------------------------------------	
Estos tres comandos se realizan conectandonos con el robot usando la terminal

TERMINAL 1:

    ssh turtlebot@192.168.1.5
    roslaunch turtlebot_bringup minimal.launch
  
TERMINAL 2:

    ssh turtlebot@192.168.1.5
    roslaunch turtlebot_bringup hokuyo_ust10lx.launch
	
TERMINAL 3:

    ssh turtlebot@192.168.1.5
    export TURTLEBOT_3D_SENSOR=astra
    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/mapa_eps.yaml

PARA MOVER EL ROBOT
--------------------------------

Estos metodos no son compatibles, por lo tanto se usara el comando de la terminal 4 con el fin de teleoperar el robot y localizarlo en rviz y una vez hecho esto la terminal se cerrara y se usaran el comando de la terminal 5 para activar el servicio de move_base.

TERMINAL 4: 

    ssh turtlebot@192.168.1.5
    roslaunch turtlebot_teleop keyboard_teleop.launch 
       
TERMINAL 5:

    python3 GoToMapPoint.py
    
TERMINAL 6:

    python3 robot_maquina_estados_gazebo_vagabundo_go_to_point_x_y.py
    
 PARA LA DETECCIÓN DE COLORES 
 --------------------------------
 
---------------------------------------------------------------------------------
METODO 1: Si se quiere usar directamente la camara del robot 

TERMINAL 5:

    python3 ColorCapture.py 
----------------------------------------------------------------------------------
MÉTODO 2: Si se quiere simular por terminal la deteccion de los colores, se puede publicar directamente en el siguiente topic. Si quiero simular que se ha detectado el color publico un 1, mientras que si quiero simular que se ha dejado de detectar publico un 0. Si después de publicar un 1 no publico un 0 el topic se quedara activado a 1 por lo que se simulara que sigue detectando un color. 
	
	rostopic pub /detect/azul std_msgs/Int16 "data: 1" 
	rostopic pub /detect/azul std_msgs/Int16 "data: 0" 
  
	rostopic pub /detect/rojo std_msgs/Int16 "data: 1" 
	rostopic pub /detect/rojo std_msgs/Int16 "data: 0"
  
 	rostopic pub /detect/verde std_msgs/Int16 "data: 1" 
	rostopic pub /detect/verde std_msgs/Int16 "data: 0" 
 
