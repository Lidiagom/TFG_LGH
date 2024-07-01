#IMPORTACIONES NECESARIAS PARA TRABAJAR CON GAZEBO OPEN CV,MEDIAPIPE HANDS, CONCURRENCIA Y OPERACIONES MATEMÁTICAS
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped #IMPORTA LOS MENSAJES CORRESPONDIENTES A LA POSICION Y VELOCIDAD DEL UAV
import cv2
import mediapipe as mp
import math
import numpy as np 
import threading 

#FUNCIONES PARA EL CONTROL EN GAZEBO
def desp(): #DESPEGUE
  pub=rospy.Publisher('/command/twist',TwistStamped,queue_size=10) #CREA UN PUBLICADOR QUE PUBLICA EN LOS COMANDOS DE VELOCIDAD
  g=TwistStamped() #DEFINE EL TIPO DE MENSAJE
  rate=rospy.Rate(10) #FRECUENCIA DE TRABAJO
  global pos_z
  flag2=0 #BANDERA PARA CONTROLAR EL FIN DEL DESPEGUE
  while(flag2!=1):  #MIENTRAS NO ESTE EN LA POSICIÓN FINAL
    lock_robot.acquire() #ASEGURO EL ACCESO A LA VARIABLE ALTURA, PARA QUE NO PUEDA SER MODIFICADA SIMULTANEAMENTE
    altura=pos_z      #ACTUALIZO LA ALTURA CON LA POSICIÓN ACTUAL DEL UAV    
    lock_robot.release()

    if altura<3:   #SI NO ALCANZA LA ALTURA DEFINIDA
      g.twist.linear.z= 0.3 #DEFINE LA VELOCIDAD EN EL EJE Z
      pub.publish(g) #PUBLICA LA VELOCIDAD
      rate.sleep()  
    else:   #SI LA ALCANZA
      g.twist.linear.z= 0.0 #ANULA LA VELOCIDAD EN EL EJE Z
      pub.publish(g) #PUBLICA LA VELOCIDAD
      flag2=1  #ACTIVA LA BANDERA DE FIN DE DESPEGUE
      rate.sleep() 
      

def aterr(): #ATERRIZAJE (2 PASOS)
    pub=rospy.Publisher('/command/twist',TwistStamped,queue_size=10) #CREA UN PUBLICADOR QUE PUBLICA EN LOS COMANDOS DE VELOCIDAD
    g=TwistStamped() #DEFINE EL TIPO DE MENSAJE
    rate=rospy.Rate(10) #FRECUENCIA DE TRABAJO
    global pos_z
    flag2=0 #BANDERA PARA CONTROLAR EL FIN DEL ATERRIZAJE
    while(flag2!=1):  #MIENTRAS NO ESTE EN LA POSICIÓN FINAL
      lock_robot.acquire() #ASEGURO EL ACCESO A LA VARIABLE ALTURA, PARA QUE NO PUEDA SER MODIFICADA SIMULTANEAMENTE
      altura=pos_z   #ACTUALIZO LA ALTURA CON LA POSICIÓN ACTUAL DEL UAV  
      lock_robot.release()

      if altura>1: #SI NO ALCANZA LA ALTURA DEFINIDA PARA EL PRIMER PASO
        g.twist.linear.z= -0.3 #DEFINE LA VELOCIDAD EN EL EJE Z (MÁS RÁPIDA)
        pub.publish(g) #PUBLICA LA VELOCIDAD
        rate.sleep()

      if altura <=1: #SI LLEGA A LA ALTURA DEFINIDA PARA EL SEGUNDO PASO DE DESCENSO
        g.twist.linear.z= -0.1 #DEFINE LA VELOCIDAD EN EL EJE Z (MÁS LENTA)
        pub.publish(g) #PUBLICA LA VELOCIDAD 
        rate.sleep()
      
      if altura<0.2: #CUANDO ESTÁ A RAS DEL SUELO (DEBIDO A DIMENSIONES DEL UAV ELEGIDO)
        g.twist.linear.z=0 #ANULA LA VELOCIDAD EN EL EJE Z
        pub.publish(g) #PUBLICA LA VELOCIDAD
        rate.sleep()
        flag2=1 #ACTIVA LA BANDERA DE FIN DE ATERRIZAJE

def hovering(): #MANTENERSE ESTÁTICO
    pub=rospy.Publisher('/command/twist',TwistStamped,queue_size=10) #CREA UN PUBLICADOR QUE PUBLICA EN LOS COMANDOS DE VELOCIDAD
    g=TwistStamped() #DEFINE EL TIPO DE MENSAJE
    g.twist.linear.x=0 #ANULA LA VELOCIDAD EN EL EJE X
    g.twist.linear.y=0 #ANULA LA VELOCIDAD EN EL EJE Y
    g.twist.linear.z=0 #ANULA LA VELOCIDAD EN EL EJE Z
    pub.publish(g) #PUBLICA LA VELOCIDAD

def pos_actual(msg): #LECTURA DE POSICION ACTUAL DEL UAV
  global pos_z
  lock_robot.acquire() #ASEGURO EL ACCESO A LA VARIABLES PARA QUE NO PUEDAN SER MODIFICADAS SIMULTANEAMENTE
  pos_x=msg.pose.position.x #ACTUALIZA LA POSICIÓN DE LA COORDENADA X DEL UAV
  pos_y=msg.pose.position.y #ACTUALIZA LA POSICIÓN DE LA COORDENADA Y DEL UAV
  pos_z=msg.pose.position.z #ACTUALIZA LA POSICIÓN DE LA COORDENADA Z DEL UAV
  lock_robot.release()

def seguimiento (Control_local, Y_gas_local) : #CONTROL DEL UAV MEDIANTE LA POSICIÓN DE LAS MANOS
  pub=rospy.Publisher('/command/twist',TwistStamped,queue_size=10) #CREA UN PUBLICADOR QUE PUBLICA EN LOS COMANDOS DE VELOCIDAD
  g=TwistStamped() #DEFINE EL TIPO DE MENSAJE
  rate=rospy.Rate(10) #FRECUENCIA DE TRABAJO
  g.twist.linear.x=Control_local[0,0]-0.5 #DEFINE LA VELOCIDAD EN EL EJE X (SI >0.5 VA HACIA LA DERECHA| SI <0.5 HACIA LA IZQUIERDA)
  g.twist.linear.y=Control_local[0,1]-0.5 #DEFINE LA VELOCIDAD EN EL EJE X (SI >0.5 SE ACERCA| SI <0.5 SE ALEJA)   
  g.twist.linear.z= ((1-(Y_gas_local/720))-0.5) #DEFINE LA VELOCIDAD EN EL EJE Z (SI >0.5 VA HACIA LA ARRIBA| SI <0.5 HACIA LA ABAJO)
  pub.publish(g) #PUBLICA LA VELOCIDAD
  rate.sleep()

def estado(): #ELECCIÓN DEL ESTADO/DETECCIÓN DEL GESTO
  global flag
  global fingers
  global Control
  global Y_gas
  global gesto
  global no_manos

  while (1):  #SIEMPRE 
    rate=rospy.Rate(10)
    if (flag!=0): #BANDERA PARA CONTROLAR QUE SE ENTRA UNA ÚNICA VEZ POR CADA VEZ QUE SE LE LLAMA
      lock_cam.acquire() #ASEGURO EL ACCESO A LA VARIABLE ALTURA, PARA QUE NO PUEDA SER MODIFICADA SIMULTANEAMENTE
      flag=0
      fingers_c=fingers #COPIA DE LOS DEDOS DETECTADOS
      fuera=0 #INDICADOR PARA SALIR DEL BUCLE DE SEGUIMIENTO

      lock_cam.release() 
      if (no_manos!=1): #COMPRUEBA LA DETECCIÓN DE MANOS
        while (fingers_c==4 and fuera==0): #SI SE TIENEN LAS DOS MANOS Y PALMAS ABIERTAS SE LLAMARÁ A SEGUIMIENTO
          lock_cam.acquire()
          fingers_c=fingers
          gesto="Seguimiento" #ACTUALIZACIÓN DEL GESTO
          if (no_manos==1): #SI PIERDE LAS MANOS EN ALGÚN MOMENTO
            publisher=hovering() #PUBLICA HOVERING
            fuera=1 #ACTIVA LA VARIABLE DE SALIDA
          lock_cam.release() 
          seguimiento(Control, Y_gas) #LLAMA A LA FUNCION DE SEGUIMIENTO 
        if fingers_c==1: #SI SE DETECTA DESPEGUE
          lock_cam.acquire()
          gesto="Despegar" #ACTUALIZACIÓN DEL GESTO
          lock_cam.release() 
          publisher=desp()  #PUBLICA DESPEGUE
        if (fingers_c==2): #SI SE DETECTA HOVERING
          lock_cam.acquire()
          gesto="Posicion fijada" # ACTUALIZACION DEL GESTO
          lock_cam.release() 
          publisher=hovering() #PUBLICA HOVERING
        if fingers_c==3: #Gesto definido como aterrizaje
          lock_cam.acquire()
          gesto="Aterrizar" # ACTUALIZACION DEL GESTO
          lock_cam.release() 
          publisher=aterr() #PUBLICA ATERRIZAJE
      else: #SI PIERDE LAS MANOS
          lock_cam.acquire()
          gesto="No manos" # ACTUALIZACION DEL GESTO
          lock_cam.release()
          publisher=hovering() #PUBLICA HOVERING
      rate.sleep()
    
def plantilla(): #DIBUJO DE LA PANTALLA
  cv2.line(frame,(640,0),(640,720),(0,0,0),2) #EJE Y
  cv2.line(frame,(200,360),(1080,360),(0,0,0),2) #EJE X
  cv2.rectangle(frame,(0,0),(200,720),(255,255,255),4) #SEPARACIÓN PARA EL EJE Z
  cv2.circle(frame,(640,360),6,(255,255,255),4) #PUNTO CENTRAL DE LOS EJES

def limites(): #FUERZA LOS LÍMITES DE LA PANTALLA (POR SI LA MANO SALE DE LA PANTALLA)
  if mano_uno [0,9]<0: #LIMITE DE LA COORDENADA X POR ABAJO 
    mano_uno[0,9]=0
  if mano_uno [0,9]>1: ##LIMITE DE LA COORDENADA X POR ARRIBA
    mano_uno[0,9]=1
  if mano_uno [1,9]<0: #LIMITE DE LA COORDENADA Y POR ABAJO 
    mano_uno[1,9]=0
  if mano_uno [1,9]>1: ##LIMITE DE LA COORDENADA Y POR ARRIBA
    mano_uno[1,9]=1

  if mano_dos [0,9]<0: #LIMITE DE LA COORDENADA X POR ABAJO 
    mano_dos[0,9]=0
  if mano_dos [0,9]>1: #LIMITE DE LA COORDENADA X POR ARRIBA 
    mano_dos[0,9]=1
  if mano_dos [1,9]<0: #LIMITE DE LA COORDENADA X POR ABAJO 
    mano_dos[1,9]=0
  if mano_dos [1,9]>1: #LIMITE DE LA COORDENADA Y POR ARRIBA 
    mano_dos[1,9]=1

#POSICIONES INICIALES DEL UAV
pos_x=0 
pos_y=0
pos_z=0
lock_robot=threading.Lock() #MUTEX
#NODO DE COMUNICACIÓN ROS
rospy.init_node('test_node')
sub=rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, pos_actual) #SUSCRIPTOR PARA OBTENER LA POSICIÓN ACTUAL DEL UAV

# CODIGO PRINCIPAL VISUALIZACIÓN
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
cap = cv2.VideoCapture(0) # ASIGNACIÓN DE LA CAMARA 
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5) #CONFIGURACIÓN DE MEDIAPIPE
lock_cam=threading.Lock() #MUTEX

flag=0 
Gas=0
Y_gas=360 
Control=np.zeros((1,2))
gesto="Gesto no detectado"
fingers=0
hilo=threading.Thread(target=estado, name='Gazebo') #CREACIÓN DEL HILO 
hilo.start() #LLAMADA AL HILO
 
# CAPTURA Y PROCESAMIENTO DE LA IMAGEN
while cv2.waitKey(1) & 0xFF != ord('c'):  #SIEMPRE ABIERTA EXCEPTO SI EL USUARIO PULSA ’C’
  j=0 #VARIABLE PARA CONTEO DE LOS 21/42 PUNTOS DE LA/LAS MANOS
  ret, frame = cap.read() # CAPTURA DE UN FOTOGRAMA
  frame=cv2.resize(frame, (1080, 720)) #REDIMENSIONA LA VENTANA DE VISUALIZAVIÓN
  frame = cv2.flip(frame, 1)# VOLTEA EL FOTOGRAMA   
  frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)#CONVERSION A MODELO RGB
  results = hands.process(frame_rgb)#PROCESAMIENTO DEL FOTOGRAMA USANDO MEDIAPIPE (ALMACENA EL VALOR DE LOS PUNTOS DETECTADOS)

  lock_cam.acquire() #ASEGURA EL ACCESO A LAS VARIABLES 
  #SI SE HAN DETECTADO MANOS, SE PROCEDE A SU ALMACENAMIENTO
  if results.multi_hand_landmarks: 
    if len(results.multi_hand_landmarks)==2: #SI SE DETECTAN DOS MANOS
      no_manos=0
    else:
     no_manos=1 #SI NO SE DETECTAN LAS DOS MANOS, NO PERMITE EL CONTROL
    mano_uno=np.zeros((2, 21)) 
    mano_dos=np.zeros((2, 21)) 
    for hand_landmarks in results.multi_hand_landmarks:#BUCLE QUE PASA POR CADA UNO DE LOS PUNTOS DETECTADOS
      mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS) #DIBUJA LOS PUNTOS Y SU UNIÓN 
      for (i,points) in enumerate(hand_landmarks.landmark): #ALMACENA EN POINTS LOS VALORES DETECTADOS
        if j <21: #RECORRE LA PRIMERA MANO
          mano_uno [0,j]= points.x #ADJUDICA LA COORDENADA X
          mano_uno [1,j]= 1-points.y #ADJUDICA LA COORDENADA Y
        else: #RECORRE LA SEGUNDA MANO
          mano_dos [0,j-21]= points.x  #ADJUDICA LA COORDENADA X
          mano_dos [1,j-21]= 1-points.y #ADJUDICA LA COORDENADA Y
        j=j+1
        if j==42: #INDICA QUE SE HAN ALMACENADO AMBAS MANOS
          j=0 #REINICIA EL CONTADOR
      
    limites() #LLAMADA A LA FUNCION PARA CORREGIR LOS VALORES LIMITROFES

    #IDENTIFICACIÓN DE MANOS Y ADJUDICACIÓN A EJES X, Y, Z
    if mano_uno[0,9]<mano_dos[0,9]: #DETECTA LA MANO QUE SE ENCUENTRA MÁS A LA IZQUIERDA (CONTROLARÁ EL EJE Z)
        Gas=mano_uno[1,9] #ADJUDICA VALOR Y DEL PUNTO CENTRAL DE LA PALMA DE LA MANO 1
        Control[0,0]=mano_dos[0,9] # ADJUDICA VALOR X DEL PUNTO CENTRAL DE LA PALMA DE LA MANO
        Control[0,1]=mano_dos[1,9] # ADJUDICA VALOR Y DEL PUNTO CENTRAL DE LA PALMA DE LA MANO
        Y_gas=(1-Gas)*720 #REDIMENSIONAMIENTO DE LA ALTURA
        fingers=0
        for i in [8,12,16,20]: #PUNTOS A COMPROBAR (SE OMITE EL PULGAR)
          if mano_dos[1,i]> mano_dos[1,i-2]: #CONTEO DE DEDOS EXTENDIDOS, COMPARANDO LA PUNTA CON LA FALANGE INFERIOR DE LA MANO QUE SE ENCUENTRA A LA DERECHA
            fingers+= 1

    else: #SI ES LA OTRA MANO LA QUE SE ENCUENTRA MAS A LA IZQUIERDA
        Gas=mano_dos[1,9] #ADJUDICA VALOR Y DEL PUNTO CENTRAL DE LA PALMA DE LA MANO 1
        Control[0,0]=mano_uno[0,9] # ADJUDICA VALOR X DEL PUNTO CENTRAL DE LA PALMA DE LA MANO
        Control[0,1]=mano_uno[1,9] # ADJUDICA VALOR Y DEL PUNTO CENTRAL DE LA PALMA DE LA MANO
        Y_gas=(1-Gas)*720 #REDIMENSIONAMIENTO DE LA ALTURA
        fingers=0
        for i in [8,12,16,20]: #PUNTOS A COMPROBAR (SE OMITE EL PULGAR)
          if mano_uno[1,i]> mano_uno[1,i-2]: #CONTEO DE DEDOS EXTENDIDOS, COMPARANDO LA PUNTA CON LA FALANGE INFERIOR DE LA MANO QUE SE ENCUENTRA A LA DERECHA
            fingers+=1

    flag=1 #ACTIVA LA BANDERA 
  
  else: #SI NO DETECTA MANOS
    no_manos=1 #ACTIVA LA VARIABLE
    cv2.putText(frame, f"No hay manos", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) #MUESTRA EN PANTALLA
  lock_cam.release()   

 #MUESTRA EN PANTALLA
  plantilla()    #LLAMADA A LA FUNCION QUE DIBUJA LOS EJES EN PANTALLA
  cv2.putText(frame, f"Controles X: {format(Control[0,0],'.3f')}", (200, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) #VALOR DE LA MANO EN EL EJE X
  cv2.putText(frame, f"          Y: {format(Control[0,1],'.3f')}", (200, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) #VALOR DE LA MANO EN EL EJE Y 
  cv2.putText(frame, f"          Z: {format(Gas,'.3f')}", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) #VALOR DE LA MANO EN EL EJE Z
  cv2.putText(frame, f"Gesto: {(gesto)}", (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) #GESTO QUE SE DETECTA
  cv2.rectangle(frame, (10,710),(190,int(Y_gas)), (0, 0, 255),20) #VALOR DE LA BARRA QUE CONTROLA LA ALTURA
  cv2.circle(frame,(int(1080*(Control[0,0])),int(720*(1-(Control[0,1])))),10,(0,0,255),5) #ULTIMO POSICIÓN DETECTADA DE LA MANO SITUADA A LA DERECHA, PARA RETOMARLA EN CASO NECESARIO 
  frame=cv2.resize(frame, (850, 500)) #REDIMENSIONAMIENTO DE LA PANTALLA QUE CONTIENE LA CAMARA
  cv2.imshow('MediaPipe Hands', frame) #MOSTRAR VENTANA 
  
# TERMINACION
cap.release() 
cv2.destroyAllWindows()