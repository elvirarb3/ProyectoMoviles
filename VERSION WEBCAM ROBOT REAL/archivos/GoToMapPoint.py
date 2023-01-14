#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped 
from std_msgs.msg import Int16, Bool
import sys

###############################################################

'''
posicion 1
Init_Positions( "map",-1.0833765268325806,0.371650755405426,0.0,0.0,0.0,0.6869488554252272, 0.7267057657883075)


header: 
  seq: 1
  stamp: 
    secs: 1673345771
    nsecs: 211151554
  frame_id: ''
goal_id: 
  stamp: 
    secs: 0
    nsecs:         0
  id: ''
goal: 
  target_pose: 
    header: 
      seq: 1
      stamp: 
        secs: 1673345927
        nsecs: 875674810
      frame_id: "map"
    pose: 
      position: 
        x: -1.0833765268325806
        y: 0.371650755405426
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.6869488554252272
        w: 0.7267057657883075
---

posicion 2

Init_Positions( "map",-3.6134350299835205,3.8924973011016846,0.0,0.0,0.0,0.9219646886644182, 0.38727395065498854)
header: 
  seq: 3
  stamp: 
    secs: 1673345867
    nsecs: 711799840
  frame_id: ''
goal_id: 
  stamp: 
    secs: 0
    nsecs:         0
  id: ''
goal: 
  target_pose: 
    header: 
      seq: 3
      stamp: 
        secs: 1673346024
        nsecs: 375634104
      frame_id: "map"
    pose: 
      position: 
        x: -3.6134350299835205
        y: 3.8924973011016846
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.9219646886644182
        w: 0.38727395065498854
posicion 3

Init_Positions( "map",-1.1530119180679321,5.972232818603516,0.0,0.0,0.0,0.3889191991780627, 0.9212718689456953)

header: 
  seq: 5
  stamp: 
    secs: 1673345954
    nsecs: 147966938
  frame_id: ''
goal_id: 
  stamp: 
    secs: 0
    nsecs:         0
  id: ''
goal: 
  target_pose: 
    header: 
      seq: 5
      stamp: 
        secs: 1673346110
        nsecs: 810503310
      frame_id: "map"
    pose: 
      position: 
        x: -1.1530119180679321
        y: 5.972232818603516
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.3889191991780627
        w: 0.9212718689456953
---

'''



##############################################################

def Init_Positions( mapa,x,y,z,ox,oy,oz, ow):
  global All_Positions
  Posicion =PoseStamped()
  Posicion.header.frame_id= mapa
  Posicion.pose.position.x= x
  Posicion.pose.position.y= y
  Posicion.pose.position.z= z 
  Posicion.pose.orientation.x= ox
  Posicion.pose.orientation.y= oy
  Posicion.pose.orientation.z= oz
  Posicion.pose.orientation.w= ow
  
  All_Positions.append(Posicion)

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')
class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, msg):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = msg.pose.position.x 
        goal.target_pose.pose.position.y = msg.pose.position.y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.x = msg.pose.orientation.x
        goal.target_pose.pose.orientation.y = msg.pose.orientation.y
        goal.target_pose.pose.orientation.z = msg.pose.orientation.z
        goal.target_pose.pose.orientation.w = msg.pose.orientation.w

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        return self.client.get_result()

#PARA PARAR LA ACCION LE MANDAMOS AL TOPIC comando STOP
#rostopic pub comando std_msgs/String STOP
def Send_Goal_To_Service(msg):
    
    cliente = ClienteMoveBase()
    result = cliente.moveTo(msg)
    pub = rospy.Publisher('goal_reach', Bool, queue_size=15)

    print("Resultado de la trayectoria ",result)
    if result:
        pub.publish(True)
        rospy.loginfo("Goal conseguido!")
        result=False
    pub.publish(False)
    rospy.loginfo("Goal NO onseguido!")

def Send_Position(msg):
    global All_Positions

    print("Nueva posicion recibida")
    Send_Goal_To_Service(All_Positions[msg.data])

def Init_Robot_Position():
    global Posicion_inicial
    Posicion_inicial.header.frame_id='map'
    Posicion_inicial.pose.pose.position.x=-1.5199998617172241
    Posicion_inicial.pose.pose.position.y= 0.13000015914440155
    Posicion_inicial.pose.pose.position.z=0
    Posicion_inicial.pose.pose.orientation.z=-5.9604637669963234e-08
    Posicion_inicial.pose.pose.orientation.w=0.9999999999999982
    Posicion_inicial.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
 
All_Positions=list() 
Posicion_inicial=PoseWithCovarianceStamped()
if __name__ == "__main__":

    ##############################
    #POSICION DE DEMO PARA VER SI FUNCIONA 
    '''
    Init_Positions( "mapa",-1.3399995565414429,1.9999998807907104,0.0,0.0,0.0,-0.073717034587982,  0.997279198024081)
    Init_Positions( "mapa",-1.6799997091293335,0.9100000262260437,0.0,0.0,0.0,-0.9273198446326869, 0.3742698301365064)

    #################################
    Init_Positions( "map",6.108769416809082,-4.870858192443848,0.0,0.0,0.0,-0.7164372323853828, 0.6976515548982694)
    Init_Positions( "map",2.465294361114502,4.49036979675293,0.0,0.0,0.0,0.9999319361206165, -0.01166718158234957)
    Init_Positions( "map",2.465294361114502,4.49036979675293,0.0,0.0,0.0,0.9999319361206165, -0.01166718158234957)
    Init_Positions( "map",-3.8167760372161865,4.962586402893066,0.0,0.0,0.0,0.9999976884622565, -0.0021501325875316977)
    Init_Positions( "map",-6.191824913024902,1.3363375663757324,0.0,0.0,0.0, -0.7123757240103247, 0.7017982814461473)
    Init_Positions( "map",1.0495561361312866,0.23933298885822296,0.0,0.0,0.0,-0.7156993112595106, 0.6984085450956784)
    print(All_Positions)  

    '''
    Init_Positions( "map",-3.6134350299835205,3.8924973011016846,0.0,0.0,0.0,0.9219646886644182, 0.38727395065498854)
    Init_Positions( "map",-1.0833765268325806,0.371650755405426,0.0,0.0,0.0,0.6869488554252272, 0.7267057657883075)
    Init_Positions( "map",-3.6134350299835205,3.8924973011016846,0.0,0.0,0.0,0.9219646886644182, 0.38727395065498854)
    Init_Positions( "map",-1.1530119180679321,5.972232818603516,0.0,0.0,0.0,0.3889191991780627, 0.9212718689456953)

    #Inicializamos la posicion del robot para que los mapas cuadren 
    Init_Robot_Position()
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=15)
    pub2 = rospy.Publisher('goal_reach', Bool, queue_size=15)

    rospy.init_node('receive_desire_position') 
    pub.publish(Posicion_inicial)
    msg=Bool()
    msg=False
    pub2.publish(msg)

    #Se recibe el indice del array de las posicion que queremos alcanzar  
    rospy.Subscriber('desire_position', Int16, Send_Position)
    #rospy.Subscriber('desire_position', PoseStamped, Send_Goal_To_Service)
    
    print('Esperando Coordenada')
    rospy.spin()

