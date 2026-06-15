import cv2
import numpy as np
import serial
import time
import threading
import os

# =====================================================
# GESTIÓN DE ARCHIVO DE CALIBRACIÓN (Puntos RBF)
# =====================================================
ARCHIVO_CALIBRACION = "calibracion_rbf.txt"

puntos_iniciales = [
    [96, 354, 25, 12], [351, 349, 125, 180], [688, 344, 153, 177], [924, 328, 164, 156],
    [50, 444, 40, 48], [236, 433, 43, 24], [545, 442, 128, 151], [957, 423, 132, 100],
    [83, 499, 72, 105], [391, 505, 100, 124], [665, 523, 112, 112], [872, 444, 123, 103],
    [914, 209, 177, 175], [73, 283, 11, 0], [116, 493, 68, 96], [275, 533, 80, 96]
]

def cargar_puntos_rbf():
    if not os.path.exists(ARCHIVO_CALIBRACION):
        np.savetxt(ARCHIVO_CALIBRACION, puntos_iniciales, fmt='%d', delimiter=',')
        print(f"📄 Archivo '{ARCHIVO_CALIBRACION}' creado con puntos base.")
    
    datos = np.loadtxt(ARCHIVO_CALIBRACION, delimiter=',')
    if datos.ndim == 1:
        datos = np.array([datos]) if datos.size > 0 else np.empty((0, 4))
    print(f"Total de puntos cargados: {len(datos)}")
    return datos

def guardar_matriz_en_disco(matriz):
    np.savetxt(ARCHIVO_CALIBRACION, matriz, fmt='%d', delimiter=',')

# Carga inicial
datos_rbf = cargar_puntos_rbf()
Puntos_Entrada = datos_rbf[:, :2] if len(datos_rbf) > 0 else np.empty((0,2))
S1_deg = datos_rbf[:, 2] if len(datos_rbf) > 0 else np.empty((0,))
S2_deg = datos_rbf[:, 3] if len(datos_rbf) > 0 else np.empty((0,))

ANCHO_MESA, ALTO_MESA  = 2850, 1550

# COORDENADAS 4 CAJAS
# =====================================================
CAJA_1_S1, CAJA_1_S2 = 138, 27   
CAJA_2_S1, CAJA_2_S2 = 144, 99   
CAJA_3_S1, CAJA_3_S2 = 54, 180   
CAJA_4_S1, CAJA_4_S2 = 63, 141   

UMBRAL_TAMANO = 3700  
UMBRAL_FR = 0.55      

# NÚCLEO MATEMÁTICO RBF
# =====================================================
EPSILON = 160.0  

def nucleo_rbf(r):
    return np.exp(-(r / EPSILON) ** 2)

def calcular_pesos_rbf(entradas, salidas):
    N = len(entradas)
    if N == 0: return np.array([])
    Matriz_A = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            distancia = np.linalg.norm(entradas[i] - entradas[j])
            Matriz_A[i, j] = nucleo_rbf(distancia)
    return np.linalg.solve(Matriz_A, salidas)

pesos_s1, pesos_s2 = np.array([]), np.array([])

def entrenar_modelo():
    global pesos_s1, pesos_s2, Puntos_Entrada, S1_deg, S2_deg
    if len(datos_rbf) > 0:
        Puntos_Entrada = datos_rbf[:, :2]
        S1_deg = datos_rbf[:, 2]
        S2_deg = datos_rbf[:, 3]
        pesos_s1 = calcular_pesos_rbf(Puntos_Entrada, S1_deg)
        pesos_s2 = calcular_pesos_rbf(Puntos_Entrada, S2_deg)
        print("🧠 Modelo RBF actualizado con éxito.")
    else:
        pesos_s1, pesos_s2 = np.array([]), np.array([])

if len(datos_rbf) > 0: entrenar_modelo()

def prediccion_rbf(px, py):
    if len(datos_rbf) == 0: return 180, 0
    punto_clic = np.array([px, py])
    s1_pred, s2_pred = 0.0, 0.0
    for i in range(len(Puntos_Entrada)):
        distancia = np.linalg.norm(punto_clic - Puntos_Entrada[i])
        s1_pred += pesos_s1[i] * nucleo_rbf(distancia)
        s2_pred += pesos_s2[i] * nucleo_rbf(distancia)
    return int(np.clip(round(s1_pred), 0, 180)), int(np.clip(round(s2_pred), 0, 180))

# CONEXIÓN SERIAL Y ÁNGULOS EN TIEMPO REAL
# =====================================================
PUERTO_COM = "COM6"

try:
    arduino = serial.Serial(PUERTO_COM, 9600, timeout=1)
    time.sleep(2.5)
    print("🔌 Conectado a Arduino en", PUERTO_COM)
except:
    arduino = None
    print("Modo Simulación activo")

HOME_S1, HOME_S2 = 180, 0
servo_actual_s1, servo_actual_s2 = HOME_S1, HOME_S2
robot_ocupado = False

# Variables de calibración interactiva por flechas
calibrando_s1 = HOME_S1
calibrando_s2 = HOME_S2

def enviar_angulos(s1, s2):
    global servo_actual_s1, servo_actual_s2
    if arduino:
        arduino.write(f"{s1},{s2}\n".encode())
    servo_actual_s1, servo_actual_s2 = s1, s2

def enviar_comando_gripper(comando):
    if arduino:
        arduino.write(f"{comando}\n".encode())

def mover_suave_rbf(s1_dest, s2_dest):
    global robot_ocupado
    robot_ocupado = True
    s1_ini, s2_ini = float(servo_actual_s1), float(servo_actual_s2)
    pasos = 35  
    for paso in range(1, pasos + 1):
        t = paso / pasos
        t_suave = (1 - np.cos(t * np.pi)) / 2 
        enviar_angulos(int(round(s1_ini + (s1_dest - s1_ini) * t_suave)), 
                       int(round(s2_ini + (s2_dest - s2_ini) * t_suave)))
        time.sleep(0.040)  
    robot_ocupado = False

# COREOGRAFÍA AUTOMÁTICA COMPLETA
# =====================================================
def ejecutar_ataque(s1, s2, tipo_objeto, area_objeto):
    def _run():
        global robot_ocupado
        robot_ocupado = True
        subtamano = "GRANDE" if area_objeto >= UMBRAL_TAMANO else "PEQUEÑO"
        
        if tipo_objeto == "TORNILLO":
            dest_s1, dest_s2 = (CAJA_1_S1, CAJA_1_S2) if subtamano == "GRANDE" else (CAJA_2_S1, CAJA_2_S2)
        else: 
            dest_s1, dest_s2 = (CAJA_3_S1, CAJA_3_S2) if subtamano == "GRANDE" else (CAJA_4_S1, CAJA_4_S2)

        mover_suave_rbf(s1, s2)
        time.sleep(0.5) 
        enviar_comando_gripper("1")
        time.sleep(0.8) 
        enviar_comando_gripper("2")
        time.sleep(0.8) 
        mover_suave_rbf(dest_s1, dest_s2)
        time.sleep(0.5)
        enviar_comando_gripper("3")
        time.sleep(2.0) 
        mover_suave_rbf(HOME_S1, HOME_S2)
        time.sleep(0.5)
        robot_ocupado = False

    threading.Thread(target=_run, daemon=True).start()

# SECUENCIA DE ARRANQUE SEGURO
# =====================================================
enviar_angulos(HOME_S1, HOME_S2)
time.sleep(4.0) 

# CONFIGURACIÓN DE VISIÓN
# =====================================================
cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

pts_src = np.float32([[1347, 492], [2580, 372], [2640, 1296], [1428, 1374]])
pts_dst = np.float32([[0, 0], [ANCHO_MESA, 0], [ANCHO_MESA, ALTO_MESA], [0, ALTO_MESA]])
M_perspectiva = cv2.getPerspectiveTransform(pts_src, pts_dst)

fondo_gris = None
capturar_nuevo_fondo = True

ultimo_x, ultimo_y = None, None
tiempo_inicio_estabilidad = None
TIEMPO_REQUERIDO_ESTABLE = 3.0  
UMBRAL_MOVIMIENTO = 10          

# CONTROL DE RATÓN INTEGRADO (CORREGIDO PARA ESCALA 1000x520)
# =====================================================
clic_x, clic_y = None, None

def raton_callback(event, x, y, flags, param):
    global clic_x, clic_y, calibrando_s1, calibrando_s2
    if event == cv2.EVENT_LBUTTONDOWN:
        # Guardamos directamente la coordenada X e Y de la ventana de 1000x520
        # Sumamos 60 en Y solo si tus 16 puntos base originales contemplaban ese desfase del recorte [60:ALTO_MESA]
        clic_x = int(x)
        clic_y = int(y + 60) 
        
        print(f" Clic registrado en la Ventana -> X: {clic_x}, Y: {clic_y}")
        # Predecimos estimación inicial con el modelo actual
        calibrando_s1, calibrando_s2 = prediccion_rbf(clic_x, clic_y)
        enviar_angulos(calibrando_s1, calibrando_s2)

cv2.namedWindow("SISTEMA CONTROL RBF DEFINITIVO")
cv2.setMouseCallback("SISTEMA CONTROL RBF DEFINITIVO", raton_callback)

while True:
    ret, frame = cap.read()
    if not ret: continue
    
    warp = cv2.warpPerspective(frame, M_perspectiva, (ANCHO_MESA, ALTO_MESA))[60:ALTO_MESA, 0:ANCHO_MESA]
    escala_x, escala_y = 1000 / warp.shape[1], 520 / warp.shape[0]
    vista = cv2.resize(warp, (1000, 520))
    
    gris = cv2.cvtColor(vista, cv2.COLOR_BGR2GRAY)
    gris = cv2.GaussianBlur(gris, (15, 15), 0)
    
    if capturar_nuevo_fondo:
        fondo_gris = gris.copy()
        capturar_nuevo_fondo = False
        continue

    diferencia = cv2.absdiff(fondo_gris, gris)
    _, mascara = cv2.threshold(diferencia, 20, 255, cv2.THRESH_BINARY)
    
    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.dilate(mascara, kernel, iterations=3)
    mascara = cv2.erode(mascara, kernel, iterations=1)
    
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    objeto_detectado = False
    x_ventana, y_ventana = 0, 0
    area_actual = 0
    clase_actual = "Ninguno"
    
    for c in contornos:
        area = cv2.contourArea(c)
        if 1000 < area < 10000:  
            M = cv2.moments(c)
            if M["m00"] != 0:
                x_ventana = int(M["m10"] / M["m00"])
                y_ventana = int(M["m01"] / M["m00"])
                objeto_detectado = True
                area_actual = area
                
                rectangulo_rotado = cv2.minAreaRect(c)
                (cx_r, cy_r), (ancho, largo), angulo_r = rectangulo_rotado
                factor_llenado_rotado = area / (ancho * largo) if (ancho * largo) > 0 else 0
                
                caja_puntos = cv2.boxPoints(rectangulo_rotado)
                caja_puntos = np.int64(caja_puntos)
                cv2.drawContours(vista, [caja_puntos], 0, (255, 255, 0), 1)

                clase_actual = "TORNILLO" if factor_llenado_rotado < UMBRAL_FR else "PERNO"
                color_clase = (0, 0, 255) if clase_actual == "TORNILLO" else (0, 255, 0)
                
                cv2.drawContours(vista, [c], -1, color_clase, 2)
                cv2.circle(vista, (x_ventana, y_ventana), 6, (0, 0, 255), -1)
                
                sub_txt = "G" if area >= UMBRAL_TAMANO else "P"
                cv2.putText(vista, f"{clase_actual}_{sub_txt} (FR:{factor_llenado_rotado:.2f})", (x_ventana - 50, y_ventana - 35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_clase, 2)
                break

    # ⏳ LÓGICA AUTOMÁTICA (Solo corre si no hay un punto seleccionado manualmente para calibrar)
    if clic_x is None and objeto_detectado and not robot_ocupado and servo_actual_s1 == HOME_S1 and servo_actual_s2 == HOME_S2:
        if ultimo_x is None or ultimo_y is None:
            ultimo_x, ultimo_y = x_ventana, y_ventana
            tiempo_inicio_estabilidad = time.time()
        else:
            distancia_movimiento = np.sqrt((x_ventana - ultimo_x)**2 + (y_ventana - ultimo_y)**2)
            if distancia_movimiento < UMBRAL_MOVIMIENTO:
                tiempo_transcurrido = time.time() - tiempo_inicio_estabilidad
                tiempo_restante = max(0.0, TIEMPO_REQUERIDO_ESTABLE - tiempo_transcurrido)
                cv2.putText(vista, f"ESTABLE: {tiempo_restante:.1f}s", (x_ventana - 50, y_ventana - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                
                if tiempo_transcurrido >= TIEMPO_REQUERIDO_ESTABLE:
                    s1, s2 = prediccion_rbf(x_ventana, y_ventana + 60)
                    ejecutar_ataque(s1, s2, clase_actual, area_actual)
                    ultimo_x, ultimo_y = None, None
                    tiempo_inicio_estabilidad = None
            else:
                ultimo_x, ultimo_y = x_ventana, y_ventana
                tiempo_inicio_estabilidad = time.time()
    else:
        if not robot_ocupado and clic_x is None:
            ultimo_x, ultimo_y = None, None
            tiempo_inicio_estabilidad = None

    # UI: Dibujar puntos morados cargados en el mapa
    for p in datos_rbf:
        cv2.circle(vista, (int(p[0]*escala_x), int((p[1]-60)*escala_y)), 5, (255, 0, 255), -1)

    # UI: Dibujar marcador de calibración activo si diste clic
    if clic_x is not None and clic_y is not None:
        cx_v, cy_v = int(clic_x * escala_x), int((clic_y - 60) * escala_y)
        cv2.circle(vista, (cx_v, cy_v), 7, (0, 255, 255), -1)
        # Cuadro informativo flotante
        cv2.rectangle(vista, (10, 10), (320, 95), (0, 0, 0), -1)
        cv2.putText(vista, f"MODO CALIBRACION ACTIVO", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(vista, f"Servos Actuales: S1={calibrando_s1}* | S2={calibrando_s2}*", (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(vista, "Flechas: Mover | S: Guardar | D: Eliminar | R: Cancelar", (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    cv2.imshow("SISTEMA CONTROL RBF DEFINITIVO", vista)

    # LÓGICA DE CAPTURA DE TECLADO INTERACTIVO (W, A, S, D)
    # =====================================================
    tecla = cv2.waitKey(1) & 0xFF  # Captura ultra rápida
    
    if tecla == 27:  # ESC para salir del programa por completo
        break
    
    elif tecla == ord('r'):  # Cancelar punto seleccionado o reiniciar fondo
        if clic_x is not None:
            clic_x, clic_y = None, None
            enviar_angulos(HOME_S1, HOME_S2)
            print(" Mapeo cancelado. Volviendo a HOME.")
        else:
            capturar_nuevo_fondo = True

    # 🕹️ MOVIMIENTO CON TECLAS UNIVERSALES (W, A, S, D)
    elif clic_x is not None and tecla == ord('w'):  # TECLA W -> Incrementa Servo 2
        calibrando_s2 = min(180, calibrando_s2 + 1)
        enviar_angulos(calibrando_s1, calibrando_s2)
        time.sleep(0.05) # Pequeña pausa para suavizar el envío de datos
        
    elif clic_x is not None and tecla == ord('s'):  # TECLA S -> Decrementa Servo 2
        calibrando_s2 = max(0, calibrando_s2 - 1)
        enviar_angulos(calibrando_s1, calibrando_s2)
        time.sleep(0.05)
        
    elif clic_x is not None and tecla == ord('a'):  # TECLA A -> Decrementa Servo 1
        calibrando_s1 = max(0, calibrando_s1 - 1)
        enviar_angulos(calibrando_s1, calibrando_s2)
        time.sleep(0.05)
        
    elif clic_x is not None and tecla == ord('d'):  # TECLA D -> Incrementa Servo 1
        calibrando_s1 = min(180, calibrando_s1 + 1)
        enviar_angulos(calibrando_s1, calibrando_s2)
        time.sleep(0.05)

    # 💾 GUARDAR NUEVO PUNTO (Ahora se activa con la tecla ENTER para no chocar con la 'D')
    elif tecla == 13 and clic_x is not None and clic_y is not None:  # 13 es el código de la tecla ENTER
        nuevo_punto = np.array([[clic_x, clic_y, calibrando_s1, calibrando_s2]])
        if len(datos_rbf) == 0:
            datos_rbf = nuevo_punto
        else:
            datos_rbf = np.vstack([datos_rbf, nuevo_punto])
        
        guardar_matriz_en_disco(datos_rbf)
        print(f" PUNTO GUARDADO CORRECTAMENTE: Mesa({clic_x},{clic_y}) -> Angulos({calibrando_s1}°, {calibrando_s2}°)")
        entrenar_modelo()
        clic_x, clic_y = None, None
        enviar_angulos(HOME_S1, HOME_S2)

    # 🗑️ ELIMINAR PUNTO EXISTENTE (Ahora se activa con la tecla BORRAR / BACKSPACE)
    elif tecla == 8 and clic_x is not None and clic_y is not None:  # 8 es el código de la tecla BACKSPACE
        if len(datos_rbf) > 0:
            distancias = np.linalg.norm(datos_rbf[:, :2] - np.array([clic_x, clic_y]), axis=1)
            indice_cercano = np.argmin(distancias)
            
            if distancias[indice_cercano] < 80:
                punto_borrado = datos_rbf[indice_cercano]
                datos_rbf = np.delete(datos_rbf, indice_cercano, axis=0)
                guardar_matriz_en_disco(datos_rbf)
                print(f"🗑️ PUNTO ELIMINADO: Mesa({int(punto_borrado[0])},{int(punto_borrado[1])})")
                entrenar_modelo()
            else:
                print("⚠️ El clic está demasiado lejos de cualquier punto morado.")
        clic_x, clic_y = None, None
        enviar_angulos(HOME_S1, HOME_S2)

cap.release()
cv2.destroyAllWindows()
if arduino: arduino.close()