#! /usr/bin/env python2

"""----------____________________
	
	Programme realiser par l'equipe de Robotique du Mali dans le concours panafriquane
	Progrmme visant a cree une solution pour la navigation d'un robot autonome
	AUTEUR :	Moctar cisse		---------->	ETUDIANT EN MEDECINE
			Amadou couclibaly	---------->
			Hamidou Sanogo		---------->	ETUDIANT EN TECHNOLAB
			Tiemmogo 		---------->	
			Ibrahim			---------->
			Diama			---------->

	Coche  :	Macleah			---------->	DIRECTEUR ROBOTSMALI	
			Tapo			---------->	
			Katikon			---------->
			Seydou			---------->

	Remercienment  : Au centre de RobotsMali qui nous ont donnes l'aide tout au long de ce concours
			 et a tout nos famille pour leurs encouragement tout au long de ce parcour

   ----------____________________	
"""			
"""|____ impport des modules necessaire qui permet l'execution du programme
    ____|"""
from multiprocessing.connection import Listener
from multiprocessing import  Process, Pipe
from threading import Thread
import time
import sqlite3
import multiprocessing as mp
import math
import cv2
import rospy
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist
import cv_bridge
import sys
import os
import datetime
from sympy import cos,sin,I,pi
from pymongo import MongoClient,errors
class Master(Process):
    """|_____Classe principale qui gere l'execution du cote serveur et controle le robot
        _____ avec les donnes recueillez par la camera et le laser
        _____ |"""
    def __init__(self,conn1,conn2,conn3,conn4,conn5):
        Process.__init__(self)
        contener = mp.Manager()
        self.dictionnaire_master_work = contener.dict()
        self.dictionnaire_master_work['laser'] = None
        self.dictionnaire_master_work['camera'] = None
        self.dictionnaire_master_work['odome'] = None
        self.dictionnaire_master_pipe = contener.list()
        self.send_laser = conn1[0]
        self.recv_laser =conn1[1]
        self.send_camera =conn2[0]
        self.recv_camera =conn2[1]
        self.send_controle = conn3[0]
        self.recv_controle = conn3[1]
        self.send_database = conn4[0]
        self.recv_database = conn4[1]
        self.send_odome = conn5[0]
        self.recv_odome = conn5[1]
        self.camera_recv = RecvThread(self.recv_camera,self.dictionnaire_master_work,'camera')
        self.laser_recv = RecvThread(self.recv_laser,self.dictionnaire_master_work,'laser')
        self.controle_recv = RecvThread(self.recv_controle,self.dictionnaire_master_work,'controle')
        self.database_recv = RecvThread(self.recv_database,self.dictionnaire_master_work,'database')
        self.odome_recv = RecvThread(self.recv_odome,self.dictionnaire_master_work,'odome')
    def run(self):
        """|____Fonction principale demarant la classe Master
            ____|"""
        #self.camera_recv.start()
        self.laser_recv.start()
        self.database_recv.start()
        self.controle_recv.start()
        self.odome_recv.start()
        for i in range(0,100):
            laser=self.dictionnaire_master_work['laser']
            #camera = self.dictionnaire_master_work['camera']
            odome = self.dictionnaire_master_work['odome']
            self.send_database.send((laser,odome))
            time.sleep(0.1)
    def center_send(self,conn,donne):
        """|____Fonction qui prend en parametre la connexion du destinateur et les donnes pour l'expedier
            ____|"""
        conn.send(donne)
    def test_master(self):
        """|____Fonction de test pour la classe master et ses methodes
            ____|"""
        self.center_send(self.send_laser,'hello laser')
        self.center_send(self.send_camera,'hello camera')
        self.center_send(self.send_controle,'hello controle')
        self.center_send(self.send_database,'hello database')     
    def recvtest(self):
        """|____Fonction de test pour envoie les donnes vers les autres modules
            ____|"""
        print(self.dictionnaire_master_pipe)
class Camera(Process):
    """|_____Classe camera charger la recuperation des donnees du camera du robot
        _____ les traites aravers les processus interne et les envoyer
        _____| vers le processus """
    def __init__(self,conn):
        """|____Fonction Principale qui initialise les variale 
            ____|"""
        Process.__init__(self)
        contener = mp.Manager()
        dictionnaire_camera_work = contener.dict()
        self.conn_send = conn[0]
        self.conn_recv = conn[1]
        self.master_recv = RecvThread(self.conn_recv,dictionnaire_camera_work,'camera')
        self.cv_bridge = cv_bridge.CvBridge()
        self.recept = rospy.Subscriber('camera/color/image_raw',Image,self.camera)
    def run(self):
        """|____Fonction principale demarant la classe camera
            ____|"""
        self.master_recv.start()
    def send_master(self,donne):
        """|____Fonction qui envoie les donnes vers le master atraver un tunnel pipe
            ____|"""
        self.conn_send.send(donne)
    def camera(self,image):
        """|____Fonction appeler pour recuperer l'image pour stoker dans le dictionnaire sans filtre
            ____|"""
        image_send = self.cv_bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        cv2.imshow(image_send)
        cv2.waitKey(1)
        #edges = cv2.resize(image_send,(100,100))
        """liste = list()
        for i in range(0,100):
            for a in range(0,100):
                b,g,r = edges[a][i]
                liste.append((int(b),int(g),int(r)))
        #cv2.imshow('test',edges)
        #cv2.waitKey(1)
        self.send_master((liste,datetime.datetime.now()))
        del liste"""
    def send_reseaus(self,image):
        self.con_reseau.send(image)
class Laser(Process):
    """_____Classe laser qui recuperer les donnes les donnees du laser
       _____ et les traites par ces processus interne et les envoies
       _____ par le thread LaserPipe vers le master
       _____|"""
    def __init__(self,conn):
        """|_____Fonction principale qui initialise la classe Laser
            _____|"""
        Process.__init__(self)
        contener = mp.Manager()
        self.dictionnaire_laser_work = contener.dict()
        self.conn_send = conn[0]
        self.conn_recv = conn[1]
        self.master_recv = RecvThread(self.conn_recv,self.dictionnaire_laser_work,'laser')
        self.recept = rospy.Subscriber('/scan',LaserScan,self.laser)
    def run (self):
        pass
    def send_master(self,donne):
        self.conn_send.send(donne)
    def laser(self,msg):
	list_laser = []
        for i in range(0,360):
                if msg.ranges[i] != float('inf'):
                    list_laser.append((round(msg.ranges[i],1),i+270))
        self.send_master((list_laser,datetime.datetime.now()))
        del list_laser
    def rotation(self,angle,rayon,regle):
        """|____Fonction qui calcul la rotation d'un point en fontion de l'angle de rotation
            ____|"""
        return (rayon*(cos(angle*pi/180) + I*sin(angle*pi/180))*(cos(pi/2)+I*sin(pi/2))).evalf()
class Database(Process):
        def __init__(self,conn):
                Process.__init__(self)
                self.url = 'localhost'
                self.port = 27017
                self.nom = 'TeamMali'
                contener = mp.Manager()
                self.dictionnaire_database_pipe = contener.dict()
                self.conn_send = conn[0]
                self.conn_recv = conn[1]
                self.master_recv = RecvThread(self.conn_recv,self.dictionnaire_database_pipe,'data')
        def run(self):
            self.master_recv.start()
            for i in range(0,100):
                print(self.dictionnaire_database_pipe)
                time.sleep(0.1)
        def connect_view(self,timeout):
                try :
                        server = MongoClient(self.url,port=self.port,serverSelectionTimeoutMS = timeout,connectTimeoutMS= timeout)
                        #test = server.list_database_names()
                        return server
                except (errors.ServerSelectionTimeoutError,AttributeError):
                        return None
        def get_col(self,server,name_col):
                if server:
                        return server[self.nom][name_col]
                return None
        def create_doc(self,server,name_col,dictionnaire):
                col = self.get_col(server,name_col)
                if col is not None:
                        return col.insert_one(dictionnaire)
                return
class Controle(Thread):
    """____classe qui s'ocupe de la vilositer du robot et de faire le trasage du robot
       ____ au debut d'executer a la fin """
    def __init__(self,conn):
        """____Fonction qui initialise la classe
           ____"""
        Thread.__init__(self)
        contener = mp.Manager()
        dictionnaire_controle_work = contener.dict()
        self.dictionnaire_controle_pipe = contener.dict()
        self.send = conn[0]
        self.recv = conn[1]
        self.master_recv = RecvThread(self.recv,self.dictionnaire_controle_pipe,'controle')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()
        self.move_cmd.linear.x=0.0  # l'acceleration sur les autres axes ne fonctionne pas
        self.move_cmd.angular.z=0.0 # la rotation sur les autres axes ne fonctionne pas
    def run(self):
        """|____Fonction principale demarant la classe controle
            ____|"""
        pass
    def send_master(self,donne):
        """|____Fonction qui permet d'envoi les donnee vers le master
            ____|"""
        self.send.send(donne)
    def testcontrole(self):
        """|____Fonction qui test la classe contole et ses methodes pour mesurer leur performance
            ____|"""
        print(self.dictionnaire_controle_pipe)
    def acceleration(self,mps,seconde=3):
        """|____Fonction qui permet d'aacelere le robot sur l'axe x en avant(positif) et
            ____ arriere(negatif)
            ____|"""
        self.now = time.time()
        print('acceleartion de ',mps,' m/s')
        self.move_cmd.linear.x=mps
        self.move_cmd.angular.z =0.0
        while time.time() -self.now<seconde:
            self.pub.publish(self.move_cmd)
            self.rate.sleep()
    def rotation(self,rps,seconde=1):
        """|____Fonction charger de la rotation du robot sur l'axe z a gauche(pisitif)
            ____ et a droite(negatif)
            ____|"""
        self.now = time.time()
        print('rotation de ',rps,' rads/s')
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z=rps
        while time.time() -self.now<seconde:
            self.pub.publish(self.move_cmd)
            self.rate.sleep()
    def arreter(self):
        """|____Fonction qui stop le robot 
            ____|"""
        print('Arrete du robot')
        self.move_cmd.linear.x=0.0
        self.move_cmd.angular.z=0.0
        self.pub.publish(self.move_cmd)
        self.rate.sleep()
class PositionProcess(Process):
    """|____CLasse qui permet de determner la position du robot au cours de son navigation
        ____|"""
    def __init__(self,conn):
            """|____Fonction qui initialise la classe de Position 
                ____|"""
            Process.__init__(self)
            rospy.Subscriber('odom',Odometry,self.odometrie)
            self.send = conn[0]
            self.recv = conn[1]
    def run(self):
            """|____Fonction demarant sa classe par l'appelation du methode du fonction start()
                ____|"""
            pass
    def odometrie(self,odom_msg):
            position= odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            vitesse_linaire= None
            vitesse_angulare=None
            pos = (round(position.x,1),round(position.y,1),round(position.z,1))
            orient= (round(orientation.x,1),round(orientation.y,1),round(orientation.z,1),round(orientation.w,1))
            dict_send = {'position' : pos,'orientation':orient}
            self.master_send((dict_send,datetime.datetime.now()))
    def master_send(self,donnee):
        self.send.send(donnee)
class RecvThread(Thread):
    """|____Classe qui permet de recevoir les donnees d'une connexion par Pipe
        ____|"""
    def __init__(self,conn,list_work,nom):
        """|____Fonction qui initialise la classe
            ____|"""
        Thread.__init__(self)
        self.conn = conn
        self.memory = list_work
        self.nom = nom
    def run(self):
        while True:
            self.memory[self.nom] = self.conn.recv()
class DeleteThread(Thread):
    def __init__(self,list_work,processus_work,rate_work):
        """|____Fonction qui initialise sa classe
            ____|"""
        Thread.__init__(self)
        self.list_work = list_work
        self.processus_work = processus_work
        self.rate_time = rate_work
    def run(self):
        while True:
            pass
class WorkProcess(Process):
    """|____Classe qui permet le traitement des donnees avec des processus et de simulation
        ____|"""
    def __init__(self,fonction,work_dict):
        """|____Fonction qui initialise la classe et ses valeur global
            ____|"""
        Process.__init__(self)
        self.fonction = fonction()
        self.memory = work_dict
    def run(self):
        self.memory = self.fonction()
if __name__ == '__main__':
    rospy.init_node('ROBOT_MALI')
    dictionnaire_parent = dict()
    dictionnaire_parent['Master_1_send'],dictionnaire_parent['laser_recv']= Pipe()
    dictionnaire_parent['Master_1_recv'],dictionnaire_parent['laser_send']= Pipe()
    dictionnaire_parent['Master_2_send'],dictionnaire_parent['camera_recv']= Pipe()
    dictionnaire_parent['Master_2_recv'],dictionnaire_parent['camera_send']= Pipe()
    dictionnaire_parent['Master_3_send'],dictionnaire_parent['controle_recv']=Pipe()
    dictionnaire_parent['Master_3_recv'],dictionnaire_parent['controle_send']=Pipe()
    dictionnaire_parent['Master_4_send'],dictionnaire_parent['database_recv']=Pipe()
    dictionnaire_parent['Master_4_recv'],dictionnaire_parent['database_send']=Pipe()
    dictionnaire_parent['Master_5_send'],dictionnaire_parent['odome_recv'] = Pipe()
    dictionnaire_parent['Master_5_recv'],dictionnaire_parent['odome_send'] = Pipe()
    dictionnaire_parent['Master_6_send'],dictionnaire_parent['connectbot_recv']= Pipe()
    dictionnaire_parent['Master_6_recv'],dictionnaire_parent['connectbot_send'] = Pipe()
    master = Master((dictionnaire_parent['Master_1_send'],dictionnaire_parent['Master_1_recv']),\
        (dictionnaire_parent['Master_2_send'],dictionnaire_parent['Master_2_recv']),\
            (dictionnaire_parent['Master_3_send'],dictionnaire_parent['Master_3_recv']),\
                (dictionnaire_parent['Master_4_send'],dictionnaire_parent['Master_4_recv']),(dictionnaire_parent['Master_5_send'],\
                dictionnaire_parent['Master_5_recv']))
    camera = Camera((dictionnaire_parent['camera_send'],dictionnaire_parent['camera_recv']))
    #laser = Laser((dictionnaire_parent['laser_send'],dictionnaire_parent['laser_recv']))
    #controle = Controle((dictionnaire_parent['controle_send'],dictionnaire_parent['controle_recv']))
    #database = Database((dictionnaire_parent['database_send'],dictionnaire_parent['database_recv']))
    #odome = PositionProcess((dictionnaire_parent['odome_send'],dictionnaire_parent['odome_recv']))
    #master.start()
    #camera.start()
    #laser.start()
    #odome.start()
    #controle.start()
    #database.start()
    rospy.spin()

    
