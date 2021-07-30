#! /usr/bin/env python2
"""|____ impport des modules necessaire qui permet l'execution du programme
    ____|"""

from multiprocessing import  Process, Pipe
#from multiprocessing.managers import SyncManager
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
import time
from geometry_msgs.msg import Twist
import cv_bridge
import sys

class Master(Process):
    """|_____Classe principale qui gere l'execution du cote serveur et controle le robot
        _____ avec les donnes recueillez par la camera et le laser
        _____ |"""
    def __init__(self,conn1,conn2,conn3,conn4):
        Process.__init__(self)
        contener = mp.Manager()
        dictionnaire_master_work = dict()

        self.dictionnaire_master_pipe = contener.dict()
        

        self.send_laser = conn1[0]
        self.recv_laser =conn1[1]
        self.send_camera =conn2[0]
        self.recv_camera =conn2[1]
        self.send_controle = conn3[0]
        self.recv_controle = conn3[1]
        self.send_database = conn4[0]
        self.recv_database = conn4[1]

        self.master_work = WorkThread(self.dictionnaire_master_pipe,dictionnaire_master_work,'master_work')

        self.camera_recv = RecvThread(self.recv_camera,self.dictionnaire_master_pipe,'camera_recv')
        self.laser_recv = RecvThread(self.recv_laser,self.dictionnaire_master_pipe,'laser_recv')
        self.controle_recv = RecvThread(self.recv_controle,self.dictionnaire_master_pipe,'controle_recv')
        self.database_recv = RecvThread(self.recv_database,self.dictionnaire_master_pipe,'database_recv')

        dictionnaire_fonction = dict()
        dictionnaire_fonction['center_send'] = self.center_send
        dictionnaire_fonction['test'] = self.test_master
        dictionnaire_fonction['testrecv'] = self.recvtest
    def run(self):
        """|____Fonction principale demarant la classe Master
            ____|"""
        init = time.time()
        self.camera_recv.start()
        self.laser_recv.start()
        self.database_recv.start()
        self.controle_recv.start()
        self.master_work.start()
        self.test_master()
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
        self.dictionnaire_camera_pipe = contener.dict()
        
        
        self.conn_send = conn[0]
        self.conn_recv = conn[1]

        self.camera_work = WorkThread(self.dictionnaire_camera_pipe,dictionnaire_camera_work,'master_recv_camera')
        self.master_recv = RecvThread(self.conn_recv,self.dictionnaire_camera_pipe,'master_recv_camera')
        self.cv_bridge = cv_bridge.CvBridge()
        self.recept = rospy.Subscriber('camera/color/image_raw',Image,self.camera)

        dictionnaire_fonction = dict()
        dictionnaire_fonction['envoie'] = self.send_master
        dictionnaire_fonction['route'] = self.route
        dictionnaire_fonction['forme'] = self.forme
        dictionnaire_fonction['pieton'] = self.pieton
        dictionnaire_fonction['test'] = self.testcam
        dictionnaire_fonction['panneau'] = self.feu_tri_color
        dictionnaire_fonction['gazon'] = self.gazon
        dictionnaire_fonction['passage_pieton'] = self.passage_pieton
        dictionnaire_fonction['but'] = self.but
        dictionnaire_fonction['mainson'] = self.mainson
    def run(self):
        """|____Fonction principale demarant la classe camera
            ____|"""
        self.master_recv.start()
        self.camera_work.start()
        
    def send_master(self,donne):
        """|____Fonction qui envoie les donnes vers le master atraver un tunnel pipe
            ____|"""
        self.conn_send.send(donne)
    def testcam(self):
        """|____Fonction de test pour la classe camera et ces different methode
            ____|"""
        print(self.dictionnaire_camera_pipe)
    def camera(self,image):
        """|____Fonction appeler pour recuperer l'image pour stoker dans le dictionnaire sans filtre
            ____|"""
        image_send = self.cv_bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        cv2.imshow('Robots_Mali',image_send)
        cv2.waitKey(1)
    def route(self):
        """|____Foction qui detect la route pour la navigation
            ____|"""
        pass
    def forme(self):
        """|____Fonction qui detect les different forme dans une image
            ____|"""
        pass
    def feu_tri_color(self):
        """|____Fonction qui detect les feux tri colors et determine leur etat
            ____|"""
        pass
    def passage_pieton(self):
        """|____Fonction qui detect le passage pour pieton
            ____|"""
        pass
    def pieton(self):
        """|____Fonction qui detect les pietons 
            ____|"""
        pass
    def gazon(self):
        """|____Fonction qui permet la detection qui gazon dans l'environement
            ____| """
        pass
    def mainson(self):
        """|____Fonction qui permet la detection des mainson dans l'environement
            ____|"""
        pass
    def but(self):
        """|____Fonction qui permet la detection du but dans l'environement
            ____|"""
        pass
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
        dictionnaire_laser_work = contener.dict()
        self.dictionnaire_laser_pipe = contener.dict()
        self.conn_send = conn[0]
        self.conn_recv = conn[1]
        self.laser_work = WorkThread(self.dictionnaire_laser_pipe,dictionnaire_laser_work,'master_recv_laser')
        self.master_recv = RecvThread(self.conn_recv,self.dictionnaire_laser_pipe,'master_recv_laser')
        self.recept = rospy.Subscriber('/scan',LaserScan,self.laser)

        dictionnaire_fonction = dict()
        dictionnaire_fonction['envoie'] = self.send_master
        dictionnaire_fonction['testlaser'] = self.testlaser
        dictionnaire_fonction['coordonner'] = self.coordonner
        dictionnaire_fonction['line'] = self.line
        dictionnaire_fonction['arc'] = self.arc
        dictionnaire_fonction['polygone'] = self.polygone
        dictionnaire_fonction['dictance'] = self.dictance
        
    def run (self):
        self.master_recv.start()
        self.laser_work.start()
        
    def send_master(self,donne):
        self.conn_send.send(donne)
    def testlaser(self):
        print(self.dictionnaire_laser_pipe)
    def laser(self,msg):
        init = time.time()
        for i in range(0,360):
            if self.coordonner(i,msg.ranges[i]) != None :
                pass
                #print(round(self.coordonne(i,msg.ranges[i])[0],3),round(self.coordonne(i,msg.ranges[i])[1],3))
        fin = time.time()
        #print('frange time execution : ',fin-init)
        
    def coordonner(self,angle,rayon):
        """|____Fonction qui renvoie les coordonnees du range du capteur laser en fonction
            ____ de l'angle et du rayon
            ____|"""
        if rayon != float('inf'):
            
            a = rayon*math.cos(-angle)
            b = rayon*math.sin(-angle)
            return (a,b)
        else :
            
            pass
        
    def line(self):
        """|____Fonction qui permet de detection une line par le capteur laser
            ____|"""
        pass
    def arc(self):
        """|____Fonction qui permet de detection un arc par le capteur laser sur le map
            ____|"""
        pass
    def polygone(self):
        """|____Fonction qui permet de detection un polygone par le capteur laser sur le map
            ____|"""
        pass
    def dictance(self):
        """|____Fonction qui permet de calculer la distance entre les formes geometrique
            ____|"""
        pass



    
class Database(Process):
    """____classe qui gere de lien entre les epreuve deja executer et les probleme
       ____ analyser par les capteur du robot
       ____"""
    def __init__(self,conn):
        """____Fonction principale qui initialise la classe
           ____"""
        Process.__init__(self)
        contener = mp.Manager()
        dictionnaire_database_work = contener.dict()
        self.dictionnaire_database_pipe = contener.dict()
        
        self.conn_send = conn[0]
        self.conn_recv = conn[1]
        self.database_work = WorkThread(self.dictionnaire_database_pipe,dictionnaire_database_work,'database_recv')
        self.master_recv = RecvThread(self.conn_recv,self.dictionnaire_database_pipe,'database_recv')
        self.conn=sqlite3.connect('ros.db')
        self.curseur=self.conn.cursor()

        dictionnaire_fonction = dict()
        dictionnaire_fonction['test'] = self.testdata
        dictionnaire_fonction['envoie'] = self.send_master
        dictionnaire_fonction['cree table'] = self.ajouteT
        dictionnaire_fonction['ajoute table'] = self.ajouteDT
        dictionnaire_fonction['supprime table'] = self.supprimeT
        dictionnaire_fonction['recherche'] =self.recherche
        dictionnaire_fonction['recherche table'] = self.affiche
        dictionnaire_fonction['modifier'] = self.modifierT
        dictionnaire_fonction['enregister'] = self.enregistrer
        dictionnaire_fonction['fermer'] = self.fermer


    def run(self):
        """|____Fonction principale demarant la classe database et les modules complementaire
            ____|"""
        self.master_recv.start()
        self.database_work.start()
        
    def testdata(self):
        """|____Fonction de test pour la classe database et ses methodes
            ____|"""
        print(self.dictionnaire_database_pipe)
    def send_master(self,donne):
        """|____Fonction qui permet d'envoi les donnes vers le master
            ____|"""
        self.conn_send.send(donne)
    def ajouteT(self,nom_de_la_table,parametre):
        """|____Focntion qui permet d'ajouter une table dans la base de donnee avec ces parametre
            ____|"""
        self.curseur.execute("create table "+nom_de_la_table+"("+parametre+")")
    
    def ajouteDT(self,nom_de_la_table,parametre,donne,para=''):
        """|____Fonction qui permet d'inserer des donnes dans une table de la base de donnee
            ____|"""
        para='?'+',?'*(len(donne)-1)
        text="insert into "+nom_de_la_table+"("+parametre+") "+"values("+para+")"
        done=donne
      
        self.curseur.execute(text,done)
    
    def supprimeT(self,nom_de_la_table):
        """|____Fonction qui supprime une table dans la base de donnee
            ____|"""
        pass
    def modifierT(self,nom_de_la_table,nom_du_donne,valeur_update):
        """____Fonction qui modifier des donnes dans une table
           ____|"""
        pass
    def rechercheT(self,nom_de_la_table,motif):
        """|____Fonction de recherche dans une table la valeur d'une donne 
            ____|"""
        pass
    def recherche(self,motif):
        """|____Fonction de recheche global dans une base de donnee en utilisant 
            ____ des thread de recherche dans chaque table la valeur d'une donnee
            ____|"""
        pass
    def enregistrer(self):
        """|____Fonction qui enregistre toute les modification de la base de donnee
            ____ initialiser 
            ____|"""
        self.conn.commit()
    def fermer(self):
        """|____Fonction qui ferme toute la connexion de la base de donnee
            ____|"""
        self.curseur.close()
        self.conn.close()
    def affiche(self,nom_de_la_table):
        """|____Fonction qui return les valeur d'une table dans la base de donnee
            ____|"""
        self.curseur.execute("select * from "+nom_de_la_table)
        self.don=list()
        for i in self.curseur:
            self.don.append(i)
            return self.don

class Controle(Process):
    """____classe qui s'ocupe de la vilositer du robot et de faire le trasage du robot
       ____ au debut d'executer a la fin """
    def __init__(self,conn):
        """____Fonction qui initialise la classe
           ____"""
        Process.__init__(self)
        contener = mp.Manager()
        dictionnaire_controle_work = contener.dict()
        self.dictionnaire_controle_pipe = contener.dict()
        
        self.send = conn[0]
        self.recv = conn[1]

        self.master_recv = RecvThread(self.recv,self.dictionnaire_controle_pipe,'controle_recv')
        self.controle_work = WorkThread(self.dictionnaire_controle_pipe,dictionnaire_controle_work,'controle_recv')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.now = time.time()
        self.move_cmd = Twist()
        self.move_cmd.linear.x=0.0  # l'acceleration sur les autres axes ne fonctionne pas
        self.move_cmd.angular.z=0.0 # la rotation sur les autres axes ne fonctionne pas

        dictionnaire_fonction = dict()
        dictionnaire_fonction['envoie'] = self.send_master
        dictionnaire_fonction['acceleration'] = self.acceleration
        dictionnaire_fonction['rotation'] = self.rotation
        dictionnaire_fonction['arreter'] = self.arreter


    def run(self):
        """|____Fonction principale demarant la classe controle
            ____|"""
        self.master_recv.start()
        self.controle_work.start()
        
    def send_master(self,donne):
        """|____Fonction qui permet d'envoi les donnee vers le master
            ____|"""
        self.send.send(donne)
    def testcontrole(self):
        """|____Fonction qui test la classe contole et ses methodes pour mesurer leur performance
            ____|"""
        print(self.dictionnaire_controle_pipe)
    def acceleration(self,mps,seconde=1):
        """|____Fonction qui permet d'aacelere le robot sur l'axe x en avant(positif) et
            ____ arriere(negatif)
            ____|"""
        self.now = time.time()
        print('acceleartion de ',mps,' m/s')
        self.move_cmd.linear.x=mps
        while time.time() -self.now<seconde:
            self.pub.publish(self.move_cmd)
            self.rate.sleep()
    def rotation(self,rps,seconde=1):
        """|____Fonction charger de la rotation du robot sur l'axe z a gauche(pisitif)
            ____ et a droite(negatif)
            ____|"""
        self.now = time.time()
        print('rotation de ',rps,' rads/s')


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
        
class SendThread(Thread):
    """____Classe qui permet d'envoyer des donnees par le tunnel Pipe
       ____"""
    def __init__(self,conn,send_dict,number,lock):
        """____Fonction qui initialise la classe"""
        Thread.__init__(self)
        self.conn = conn
        self.lock = lock
    def run(self):
            
        for i in range(0,number):
            self.conn.send(send_dict)

class RecvThread(Thread):
    """|____Classe qui permet de recevoir les donnees d'une connexion par Pipe
        ____|"""
    def __init__(self,conn,work_dict,nom):
        """|____Fonction qui initialise la classe
            ____|"""
        Thread.__init__(self)
        self.conn = conn
        self.memory = work_dict
        self.nom =nom
        
        
    def run(self):
        
         self.memory[self.nom] = self.conn.recv()
            
        
class WorkThread(Thread):
    """|____Classe qui permet le traitement des donnees avec les thread et de similation
        ____|"""
    def __init__(self,dictionnaire_suivu,work_dict,nom):
        """|____Fonction qui initialise la classe 
            ____|"""
        Thread.__init__(self)
        self.dictionnaire_pipe = dictionnaire_suivu
        self.memory = work_dict
        dictionnaire = mp.Manager()
        dictionnaire = dictionnaire.dict()
        self.nom = nom
        
        
    def run(self):
        """|____Fonction demarant la classe workthread
            ____|"""
        try :
            print(self.dictionnaire_pipe)
        except :
            pass
    def test_fonction(self):
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
    def test_fonction(self,fonction):
        pass

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
    master = Master((dictionnaire_parent['Master_1_send'],dictionnaire_parent['Master_1_recv']),\
        (dictionnaire_parent['Master_2_send'],dictionnaire_parent['Master_2_recv']),\
            (dictionnaire_parent['Master_3_send'],dictionnaire_parent['Master_3_recv']),\
                (dictionnaire_parent['Master_4_send'],dictionnaire_parent['Master_4_recv']))
    camera = Camera((dictionnaire_parent['camera_send'],dictionnaire_parent['camera_recv']))
    laser = Laser((dictionnaire_parent['laser_send'],dictionnaire_parent['laser_recv']))
    controle = Controle((dictionnaire_parent['controle_send'],dictionnaire_parent['controle_recv']))
    database = Database((dictionnaire_parent['database_send'],dictionnaire_parent['database_recv']))
    master.start()
    camera.start()
    laser.start()
    
    controle.start()
    database.start()
    """master.join()
    camera.join()
    laser.join()
    controle.join()
    database.join()"""
    rospy.spin()

    
