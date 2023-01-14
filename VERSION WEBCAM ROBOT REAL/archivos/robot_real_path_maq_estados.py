#!/usr/bin/env python

import rospy
import smach
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool
from nav_msgs.msg import Odometry
import math
import time 

mover = 1
w_act = 0


# CODIGO ROBOT VAGABUNDO

        
class Vagabundo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parar'], output_keys=['salida_v'])
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_vaga)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.mov=Twist()
    
    def execute(self, userdata):
        rospy.loginfo('Ejecutando estado VAGABUNDO')
        userdata.salida_v = 1
        while(1):
            tiem=0
            while tiem<100:
                tiem=tiem+1
                self.pub.publish(self.mov)
                time.sleep(.1)
            return 'parar'
    def callback_vaga(self,msg):
            global mover
        # leer sensor laser
        # rango = 1080
            #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
            
            rango = (msg.angle_max -  msg.angle_min)/ msg.angle_increment
            cont = 0
            lado = rango/5
            izquierda = 10000
            derecha = 10000
            centro = 10000

        # calcular valor minimo der, izq y centro
            for lectura in msg.ranges:
                if cont < lado:
                    if lectura < derecha: 
                        derecha = lectura
                    
                elif cont > (2*lado) and cont < (3*lado):
                    if lectura < centro: 
                        centro = lectura

                elif cont > (rango - lado):
                    if lectura < izquierda: 
                        izquierda = lectura

                cont += 1

            #print ('derecha = ', derecha, 'izquierda = ', izquierda, 'centro = ', centro)
        # print ('\n')
            mov = Twist()

        # determinar obstaculos y mov robot para evitarlos
        # determinar obstaculos y mov robot para evitarlos
        # vel + --> izquierda
            obst = 1.0
            colision = 0.5
            vel = 0.3
            

        # si se va a chocar, no avanzar  
            if mover == 0 or centro < colision: 
                mov.linear.x = 0
                mover = 0
            else:
                mov.linear.x = 0.2+0.3*centro/5
                
        # evitar obstaculos mas cercanos 
            if derecha < izquierda:
                if derecha < obst:
                    mov.angular.z = -vel - (obst + derecha)*0.1
            
            elif izquierda < derecha:
                if izquierda < obst:
                    mov.angular.z = vel +(obst - izquierda)*0.1

        # si parado, girar a izquierda
            if mov.linear.x == 0:
                mov.angular.z = vel + (colision - centro)*0.1
                if (centro > 0.8):
                    mover = 1
                else: 
                    mover = 0

            #print ('mov lineal = ', mov.linear.x, 'mov ang = ', mov.angular.z)
            #print ('\n')
            #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
            self.mov=mov
            #self.pub.publish(mov)
                

# definir estado Parar
class Senyal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['derecha', 'izquierda', 'parar', 'bucle'], output_keys=['salida_s'])
        
        self.azul=0
        self.rojo=0
        self.verde=0
        self.blue = rospy.Subscriber('/detect/azul', Int16, self.azules)
        self.red= rospy.Subscriber('/detect/rojo', Int16, self.rojos)
        self.green= rospy.Subscriber('/detect/verde', Int16, self.verdes)
    
    # COLORES CANALES
    def azules(self,msg):
        if msg.data == 1:
            self.azul = 1
        else:
            self.azul = 0


    def rojos(self,msg):
        if msg.data == 1:
            self.rojo = 1
        else:
            self.rojo = 0

    def verdes(self,msg):
        if msg.data == 1:
            self.verde= 1
        else:
            self.verde = 0
        

    def execute(self, userdata):
        rospy.loginfo('Ejecutando estado SENYAL')
        userdata.salida_s = 2      
        
        tiem=0
        while tiem<100:
            if self.azul == 1:
                return 'derecha'
            elif self.rojo == 1:
                return 'izquierda'
            elif self.verde == 1:
                return 'parar' 
            tiem=tiem+1
            time.sleep(.1)
        return 'bucle'
        


class Derecha(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['vagabundo'])
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_der)
        self.odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.mov=Twist()

    # ODOMETRIA
    def callback_odom(self, msg):
        global x_act, y_act, w_act
        x_act = msg.pose.pose.position.x
        y_act = msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t1 = 2.0*(w*z+x*y)
        t2 = 1.0-2.0*(y*y+z*z)
        w_act =math.atan2(t1,t2)

    def execute(self, userdata):
        rospy.loginfo('Ejecutando estado DERECHA')
        global w_act
        mov = Twist()
        # comandos de colores 
        mov.linear.x = 0
        mov.angular.z = 0.2
        w_antes = w_act
        while (abs(abs(w_antes)-abs(w_act)) < 1.57):
            self.pub.publish(mov)
        # print "w_antes = ", abs(abs(w_antes)-abs(w_act)), "   w_act = ", w_act
        mov.angular.z = 0
        self.pub.publish(mov)
        self.pub.publish(self.mov)

        return 'vagabundo'

class Izquierda(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['vagabundo'])
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_izq)
        self.odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.mov=Twist()
        
    def callback_odom(self, msg):
        global x_act, y_act, w_act
        x_act = msg.pose.pose.position.x
        y_act = msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t1 = 2.0*(w*z+x*y)
        t2 = 1.0-2.0*(y*y+z*z)
        w_act =math.atan2(t1,t2)

    def execute(self, userdata):
        rospy.loginfo('Ejecutando estado IZQUIERDA')
        global w_act
        mov = Twist()
        # comandos de colores 
        mov.linear.x = 0
        mov.angular.z = -0.2
        w_antes = w_act
        while (abs(abs(w_antes)-abs(w_act)) < 1.57):
            self.pub.publish(mov)
            #print "w_antes = ", abs(abs(w_antes)-abs(w_act)), "   w_act = ", w_act
        mov.angular.z = 0
        self.pub.publish(mov)
        return 'vagabundo'


class Parar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['vagabundo', 'senyal'], input_keys=['entrada'])
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_parar)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.mov=Twist()
        
    def callback_parar(self,msg):
    # comandos de colores 
        mov = Twist()
        mov.linear.x = 0
        mov.angular.z = 0.0
        i = 0
        self.mov=mov

    def execute(self, userdata):
        self.pub.publish(self.mov)
        time.sleep(.1)
        rospy.loginfo('Ejecutando estado PARAR')
        if userdata.entrada == 1:
            return 'senyal'
        else:
            return 'vagabundo'
    
class Path_Planning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['senyal' ,'taskcomplete'])
        self.pub = rospy.Publisher('desire_position', Int16, queue_size=15)
        self.sub=rospy.Subscriber('goal_reach', Bool, self.callback)
        self.estado=False
        self.indice=0
        self.uno=True
    
    def callback(self,msg):
        if msg.data ==True:
            self.estado=True

    def execute(self, userdata):
        while self.indice<3:
            time.sleep(.1)
            while self.estado==False:
                if self.uno==True:
                    print("enviando... ", self.indice)
                    self.pub.publish(self.indice)
                    self.uno=False
            if self.estado==True:
                self.indice=self.indice +1
                self.estado=False
                self.uno=True
                print("Inidice actual antes de cambiar de estado", self.indice)
                return 'senyal'
        return 'taskcomplete'
        

# main
def main():
    rospy.init_node('maquina_estados')

    # Crear maquina de estados SMACH
    sm = smach.StateMachine(outcomes=['HECHO', 'colision'])

    # Abrir maquina estados
    with sm:
        # Incluir estados en maquina
        smach.StateMachine.add('PATH', Path_Planning(), transitions={'senyal':'SENYAL' ,'taskcomplete':'HECHO'})
        smach.StateMachine.add('VAGABUNDO', Vagabundo(), transitions={'parar':'PARAR'}, remapping={'salida_v':'stop'})
        smach.StateMachine.add('SENYAL', Senyal(), transitions={'derecha':'DERECHA', 'izquierda':'IZQUIERDA', 'parar':'PARAR', 'bucle':'SENYAL'}, remapping = {'salida_s':'stop'})
        smach.StateMachine.add('DERECHA', Derecha(), transitions={'vagabundo':'PATH'})
        smach.StateMachine.add('IZQUIERDA', Izquierda(), transitions={'vagabundo':'PATH'})
        smach.StateMachine.add('PARAR', Parar(), transitions={'vagabundo':'PATH', 'senyal':'SENYAL'}, remapping={'entrada':'stop'})
        
        # Ejecutar maquina estados
        outcome = sm.execute()

if __name__ == '__main__':
    main()
