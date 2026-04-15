import cv2
import numpy as np
import math
from picamera2 import Picamera2
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================
VEL_BASE_MAX = 0.50
VEL_BASE_MIN = 0.17

Kp_ANG = 0.012
Kp_ERR = 0.14
MAX_CORR = 0.26
ANG_90 = 45

# ============================================================
# PIVOT ADAPTATIVO
# ============================================================
PIVOT_ANG_ON = 26
PIVOT_ANG_OFF = 16
PIVOT_ERR_ON = 72
PIVOT_ERR_OFF = 36

MIN_PWM_PIVOT = 0.23
BOOST_PIVOT = 0.05
PIVOT_VEL_MAX = 0.35

# ============================================================
# GIROS NORMALES
# ============================================================
VEL_GIRO_NORMAL = 0.25
TIEMPO_GIRO_NORMAL_IZQ = 0.20
TIEMPO_GIRO_NORMAL_DER = 0.20
UMBRAL_ANG_GIRO_NORMAL = 34
COOLDOWN_GIRO_NORMAL = 0.28

# ============================================================
# GIROS POR VERDE
# ============================================================
VEL_GIRO_VERDE = 0.30
VEL_GIRO_VERDE_U = 0.31

VEL_AVANCE_VERDE = 0.14
VEL_AVANCE_U = 0.12

TIEMPO_AVANCE_VERDE = 2.8
TIEMPO_AVANCE_U = 0.50

TIEMPO_GIRO_VERDE_IZQ = 3.00
TIEMPO_GIRO_VERDE_DER = 2.5
TIEMPO_GIRO_VERDE_U = 6.00

COOLDOWN_VERDE = 0.95

# ============================================================
# PAUSA / RELECTURA PARA VERDE
# ============================================================
PAUSA_VERDE_TIEMPO = 0.30
PAUSA_VERDE_FRAMES_CONFIRM = 6

VEL_RETROCESO_VERDE = 0.12
TIEMPO_RETROCESO_VERDE = 0.18

VEL_RELECTURA_VERDE = 0.10
TIEMPO_RELECTURA_VERDE = 0.28

# ============================================================
# GAP / RECUPERACIÓN DE LÍNEA
# ============================================================
GAP_CONFIRM_FRAMES = 2
GAP_FORWARD_TIME = 1.00
VEL_GAP_FORWARD = 0.50
GAP_EXIT_TO_CENTER = True

# ============================================================
# NUEVA RUTA PARA VERDE DIFÍCIL / U APRETADA
# ============================================================
VERDE_REQUIERE_REPOSICION_SI_ERRX = 25
VERDE_REQUIERE_REPOSICION_SI_ANG = 16
VERDE_REPOSICIONAR_SI_DET_INF_NONE = True

VEL_PREP_VERDE_REV = 0.10
TIME_PREP_VERDE_REV = 0.18

VEL_REPO_GIRO = 0.18
VEL_REPO_MICRO = 0.11
VEL_REPO_BACK = 0.07
TIMEOUT_REPOSICION_VERDE = 0.55

REPO_VERDE_ERR_OK = 14
REPO_VERDE_ANG_OK = 10
REPO_VERDE_FRAMES_OK = 4

# ============================================================
# CURVA CERRADA REVERSA
# ============================================================
CURVA_REV_ANG_ON = 40
CURVA_REV_ANG_OK = 10
CURVA_REV_ERR_OK = 12
CURVA_REV_FRAMES_OK = 6
CURVA_REV_TIMEOUT = 1.40

CURVA_REV_PIVOT_VEL = 0.31
CURVA_REV_BACK_VEL = 0.14

CURVA_REV_PIVOT_1_TIME = 0.13
CURVA_REV_BACK_TIME = 0.09
CURVA_REV_PIVOT_2_TIME = 0.11

# ============================================================
# CENTRADO SOBRE EJE DESPUÉS DE GIRO NORMAL
# ============================================================
EJE_ANG_OK = 7
EJE_ERR_OK = 9
EJE_FRAMES_OK = 6
EJE_TIMEOUT = 1.00
VEL_EJE_GIRO = 0.24

# ============================================================
# ÁREAS MÍNIMAS
# ============================================================
MIN_AREA_VERDE = 170
MIN_AREA_LINEA = 110
MIN_AREA_ROJO = 180

# ============================================================
# VALIDACIÓN DE VERDE / INTERSECCIÓN
# ============================================================
FACTOR_ANCHO_SUP_INF = 1.30
DIF_MIN_ANCHO = 16
TOLERANCIA_ANCHO_PARECIDO = 12
DIF_MIN_SUP_MAYOR_INF = 14

FRACCION_Y_PREVIEW_MIN = 0.16
FRACCION_Y_PREVIEW_MAX = 0.58
FRACCION_Y_DECISION_MIN = 0.56
FRACCION_Y_DECISION_MAX = 0.90

TIMEOUT_VERDE_PENDIENTE = 0.75
TOL_X_CENTRO = 10

# ============================================================
# DETECCIÓN DE U
# ============================================================
TOL_VERDE_U_Y = 30
TOL_VERDE_U_X = 36
TOL_VERDE_U_AREA = 160

# ============================================================
# PROTECCIONES
# ============================================================
VERDE_CONFIRM_FRAMES = 1
TIEMPO_BLOQUEO_VERDE_POSTGIRO = 0.65
TIEMPO_BLOQUEO_VERDE_U = 1.5

# ============================================================
# CENTRADO DESPUÉS DE GIROS
# ============================================================
CENTRAR_ERR_MAX = 8
CENTRAR_ANG_MAX = 9
CENTRAR_FRAMES_OK = 10

VEL_CENTRAR_GIRO = 0.22
VEL_CENTRAR_MICRO = 0.13
VEL_CENTRAR_AVANCE = 0.08

TIMEOUT_CENTRADO = 1.35

# ============================================================
# ESTABILIZACIÓN DESPUÉS DE GIRO
# ============================================================
POST_GIRO_ANG_OK = 6
POST_GIRO_ERR_OK = 8
POST_GIRO_FRAMES_OK = 8
POST_GIRO_TIMEOUT = 1.00

VEL_POST_MAX = 0.16
VEL_POST_MIN = 0.10

# ============================================================
# BLOQUEO DE VERDE POR SALIDA DE DIAGONAL
# ============================================================
ANGULO_MAX_VERDE_RECTO = 20
ERRX_MAX_VERDE_RECTO = 30
FRAMES_RECTO_PARA_VERDE = 3

# ============================================================
# PREPARAR VERDE ANTES DE GIRAR
# ============================================================
PREP_VERDE_TIMEOUT = 0.20
PREP_VERDE_ANG_OK = 8
PREP_VERDE_ERR_OK = 16
PREP_VERDE_FRAMES_OK = 3
VEL_PREP_VERDE_MAX = 0.35
VEL_PREP_VERDE_MIN = 0.20

# ============================================================
# GPIO MOTORES
# ============================================================
AIN1_M1 = DigitalOutputDevice(27)
AIN2_M1 = DigitalOutputDevice(17)
PWMA_U1 = PWMOutputDevice(18)

BIN1_M2 = DigitalOutputDevice(5)
BIN2_M2 = DigitalOutputDevice(6)
PWMB_U1 = PWMOutputDevice(19)

AIN1_M3 = DigitalOutputDevice(21)
AIN2_M3 = DigitalOutputDevice(20)
PWMA_U2 = PWMOutputDevice(12)

BIN1_M4 = DigitalOutputDevice(16)
BIN2_M4 = DigitalOutputDevice(25)
PWMB_U2 = PWMOutputDevice(13)

STBY1 = DigitalOutputDevice(22)
STBY2 = DigitalOutputDevice(26)

STBY1.on()
STBY2.on()

# ============================================================
# AJUSTE FINO DE MOTORES
# ============================================================
INV_M1 = 1
INV_M2 = 1
INV_M3 = 1
INV_M4 = 1

DEADZONE_M1 = 0.00
DEADZONE_M2 = 0.00
DEADZONE_M3 = 0.00
DEADZONE_M4 = 0.00

# ============================================================
# FUNCIONES DE MOTOR
# ============================================================
def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def preparar_senal(valor, inversion=1, deadzone=0.0):
    valor = clamp(valor) * inversion
    if abs(valor) < 1e-6:
        return 0.0

    pwm = abs(valor)
    pwm = deadzone + pwm * (1.0 - deadzone)
    pwm = min(pwm, 1.0)
    return pwm if valor > 0 else -pwm


def aplicar_motor(valor, in1, in2, pwm):
    if abs(valor) < 1e-6:
        in1.off()
        in2.off()
        pwm.value = 0
        return

    pwm.value = abs(valor)
    in1.value = valor < 0
    in2.value = valor > 0


def stop_motores():
    aplicar_motor(0, AIN1_M1, AIN2_M1, PWMA_U1)
    aplicar_motor(0, BIN1_M2, BIN2_M2, PWMB_U1)
    aplicar_motor(0, AIN1_M3, AIN2_M3, PWMA_U2)
    aplicar_motor(0, BIN1_M4, BIN2_M4, PWMB_U2)


def motores(m1, m2, m3, m4):
    s1 = preparar_senal(m1, INV_M1, DEADZONE_M1)
    s2 = preparar_senal(m2, INV_M2, DEADZONE_M2)
    s3 = preparar_senal(m3, INV_M3, DEADZONE_M3)
    s4 = preparar_senal(m4, INV_M4, DEADZONE_M4)

    aplicar_motor(s1, AIN1_M1, AIN2_M1, PWMA_U1)
    aplicar_motor(s2, BIN1_M2, BIN2_M2, PWMB_U1)
    aplicar_motor(s3, AIN1_M3, AIN2_M3, PWMA_U2)
    aplicar_motor(s4, BIN1_M4, BIN2_M4, PWMB_U2)


def avanzar(v):
    motores(v, v, v, v)


def retroceder(v):
    motores(-v, -v, -v, -v)


def girar_izquierda(v):
    v = max(abs(v) + BOOST_PIVOT, MIN_PWM_PIVOT)
    v = min(v, PIVOT_VEL_MAX)
    motores(v, -v, v, -v)


def girar_derecha(v):
    v = max(abs(v) + BOOST_PIVOT, MIN_PWM_PIVOT)
    v = min(v, PIVOT_VEL_MAX)
    motores(-v, v, -v, v)


def giro_reversa_izquierda(v_giro, v_back):
    motores(-v_back, v_giro, -v_back, v_giro)


def giro_reversa_derecha(v_giro, v_back):
    motores(v_giro, -v_back, v_giro, -v_back)

# ============================================================
# CÁMARA
# ============================================================
cam = Picamera2()
cam.configure(cam.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
))
cam.start()
time.sleep(1)

# ============================================================
# TRACKBARS
# ============================================================
def nothing(x):
    pass

cv2.namedWindow("Calibracion Negro")
cv2.createTrackbar("L-H", "Calibracion Negro", 0, 180, nothing)
cv2.createTrackbar("L-S", "Calibracion Negro", 0, 255, nothing)
cv2.createTrackbar("L-V", "Calibracion Negro", 0, 255, nothing)
cv2.createTrackbar("U-H", "Calibracion Negro", 180, 180, nothing)
cv2.createTrackbar("U-S", "Calibracion Negro", 255, 255, nothing)
cv2.createTrackbar("U-V", "Calibracion Negro", 85, 255, nothing)

cv2.namedWindow("Calibracion Verde")
cv2.createTrackbar("L-H", "Calibracion Verde", 35, 180, nothing)
cv2.createTrackbar("L-S", "Calibracion Verde", 131, 255, nothing)
cv2.createTrackbar("L-V", "Calibracion Verde", 0, 255, nothing)
cv2.createTrackbar("U-H", "Calibracion Verde", 95, 180, nothing)
cv2.createTrackbar("U-S", "Calibracion Verde", 255, 255, nothing)
cv2.createTrackbar("U-V", "Calibracion Verde", 255, 255, nothing)

cv2.namedWindow("Calibracion Rojo")
cv2.createTrackbar("L1-H", "Calibracion Rojo", 0, 180, nothing)
cv2.createTrackbar("L1-S", "Calibracion Rojo", 120, 255, nothing)
cv2.createTrackbar("L1-V", "Calibracion Rojo", 175, 255, nothing)
cv2.createTrackbar("U1-H", "Calibracion Rojo", 20, 180, nothing)
cv2.createTrackbar("U1-S", "Calibracion Rojo", 255, 255, nothing)
cv2.createTrackbar("U1-V", "Calibracion Rojo", 233, 255, nothing)

cv2.createTrackbar("L2-H", "Calibracion Rojo", 59, 180, nothing)
cv2.createTrackbar("L2-S", "Calibracion Rojo", 120, 255, nothing)
cv2.createTrackbar("L2-V", "Calibracion Rojo", 80, 255, nothing)
cv2.createTrackbar("U2-H", "Calibracion Rojo", 180, 180, nothing)
cv2.createTrackbar("U2-S", "Calibracion Rojo", 255, 255, nothing)
cv2.createTrackbar("U2-V", "Calibracion Rojo", 255, 255, nothing)

# ============================================================
# FUNCIONES AUXILIARES
# ============================================================
def leer_hsv(nombre_ventana):
    lh = cv2.getTrackbarPos("L-H", nombre_ventana)
    ls = cv2.getTrackbarPos("L-S", nombre_ventana)
    lv = cv2.getTrackbarPos("L-V", nombre_ventana)
    uh = cv2.getTrackbarPos("U-H", nombre_ventana)
    us = cv2.getTrackbarPos("U-S", nombre_ventana)
    uv = cv2.getTrackbarPos("U-V", nombre_ventana)
    return (lh, ls, lv), (uh, us, uv)


def leer_hsv_rojo():
    l1h = cv2.getTrackbarPos("L1-H", "Calibracion Rojo")
    l1s = cv2.getTrackbarPos("L1-S", "Calibracion Rojo")
    l1v = cv2.getTrackbarPos("L1-V", "Calibracion Rojo")
    u1h = cv2.getTrackbarPos("U1-H", "Calibracion Rojo")
    u1s = cv2.getTrackbarPos("U1-S", "Calibracion Rojo")
    u1v = cv2.getTrackbarPos("U1-V", "Calibracion Rojo")

    l2h = cv2.getTrackbarPos("L2-H", "Calibracion Rojo")
    l2s = cv2.getTrackbarPos("L2-S", "Calibracion Rojo")
    l2v = cv2.getTrackbarPos("L2-V", "Calibracion Rojo")
    u2h = cv2.getTrackbarPos("U2-H", "Calibracion Rojo")
    u2s = cv2.getTrackbarPos("U2-S", "Calibracion Rojo")
    u2v = cv2.getTrackbarPos("U2-V", "Calibracion Rojo")

    low1 = (l1h, l1s, l1v)
    high1 = (u1h, u1s, u1v)
    low2 = (l2h, l2s, l2v)
    high2 = (u2h, u2s, u2v)

    return low1, high1, low2, high2


def limpiar_mascara(mask, erode_iter=1, dilate_iter=2):
    mask = cv2.erode(mask, None, iterations=erode_iter)
    mask = cv2.dilate(mask, None, iterations=dilate_iter)
    return mask


def detectar_objeto_principal(mask, y_offset=0, min_area=100):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    x, y, w, h = cv2.boundingRect(c)

    return {
        "contorno": c,
        "area": area,
        "x": x, "y": y, "w": w, "h": h,
        "cx_local": cx_local,
        "cy_local": cy_local,
        "cx_global": cx_local,
        "cy_global": cy_local + y_offset,
        "ancho": w,
        "alto": h
    }


def detectar_verdes(mask_verde, y_offset, min_area=150):
    detecciones = []
    contours, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue

        cx_local = int(M["m10"] / M["m00"])
        cy_local = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(c)

        detecciones.append({
            "area": area,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx_global": cx_local,
            "cy_global": cy_local + y_offset
        })

    return detecciones


def dibujar_det_linea(frame, det, y_offset, color, texto):
    x = det["x"]
    y = det["y"] + y_offset
    w = det["w"]
    h = det["h"]
    cx = det["cx_global"]
    cy = det["cy_global"]

    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    cv2.circle(frame, (cx, cy), 5, color, -1)
    cv2.putText(frame, f"{texto} ({cx},{cy})", (x, max(y - 6, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)


def dibujar_det_verde(frame, det, etiqueta=None):
    x = det["x"]
    y = det["y"]
    w = det["w"]
    h = det["h"]
    cx = det["cx_global"]
    cy = det["cy_global"]

    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    if etiqueta is not None:
        cv2.putText(frame, etiqueta, (x, max(y - 6, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)


def velocidad_pivot(angle, error_x):
    v = 0.22 + abs(angle) * 0.0034 + abs(error_x) * 0.0011
    v = max(MIN_PWM_PIVOT, v)
    v = min(PIVOT_VEL_MAX, v)
    return v


def en_banda_vertical(cy, y1, y2, fmin, fmax):
    alto = y2 - y1
    y_rel = cy - y1
    return (alto * fmin) <= y_rel <= (alto * fmax)


def seleccionar_verdes_candidatos(detecciones, cx_ref, y1_verde, y2_verde, fmin, fmax):
    candidatos = []

    for det in detecciones:
        cx = det["cx_global"]
        cy = det["cy_global"]

        if not en_banda_vertical(cy, y1_verde, y2_verde, fmin, fmax):
            continue

        if cy > (y1_verde + 0.82 * (y2_verde - y1_verde)):
            continue

        if abs(cx - cx_ref) <= TOL_X_CENTRO:
            continue

        lado = "IZQ" if cx < cx_ref else "DER"

        candidatos.append({
            "lado": lado,
            "cx": cx,
            "cy": cy,
            "area": det["area"],
            "det": det
        })

    candidatos.sort(key=lambda d: (d["cy"], -d["area"]))
    return candidatos


def decidir_accion_verde(candidatos, cx_ref):
    if not candidatos:
        return None, None

    verdes_izq = [c for c in candidatos if c["lado"] == "IZQ"]
    verdes_der = [c for c in candidatos if c["lado"] == "DER"]

    mejor_izq = verdes_izq[0] if verdes_izq else None
    mejor_der = verdes_der[0] if verdes_der else None

    if mejor_izq and not mejor_der:
        return "IZQ", {"tipo": "simple_izq", "izq": mejor_izq, "der": None}

    if mejor_der and not mejor_izq:
        return "DER", {"tipo": "simple_der", "izq": None, "der": mejor_der}

    if mejor_izq and mejor_der:
        dy = abs(mejor_izq["cy"] - mejor_der["cy"])
        dx_sim = abs(abs(mejor_izq["cx"] - cx_ref) - abs(mejor_der["cx"] - cx_ref))
        area_ok = mejor_izq["area"] >= TOL_VERDE_U_AREA and mejor_der["area"] >= TOL_VERDE_U_AREA

        if dy <= TOL_VERDE_U_Y and dx_sim <= TOL_VERDE_U_X and area_ok:
            return "U", {"tipo": "u", "izq": mejor_izq, "der": mejor_der}

        if mejor_izq["cy"] < mejor_der["cy"]:
            return "IZQ", {"tipo": "prioridad_arriba_izq", "izq": mejor_izq, "der": mejor_der}
        elif mejor_der["cy"] < mejor_izq["cy"]:
            return "DER", {"tipo": "prioridad_arriba_der", "izq": mejor_izq, "der": mejor_der}

        if mejor_izq["area"] >= mejor_der["area"]:
            return "IZQ", {"tipo": "empate_area_izq", "izq": mejor_izq, "der": mejor_der}
        else:
            return "DER", {"tipo": "empate_area_der", "izq": mejor_izq, "der": mejor_der}

    return None, None


def accion_verde_texto(accion):
    if accion == "IZQ":
        return "Verde IZQ"
    if accion == "DER":
        return "Verde DER"
    if accion == "U":
        return "Verde U"
    return "Sin verde"


def score_interseccion(ancho_sup, ancho_mid, ancho_inf, det_sup, det_mid, det_inf):
    score = 0
    razones = []

    if ancho_sup is not None and ancho_inf is not None:
        dif = ancho_sup - ancho_inf

        if ancho_sup > ancho_inf * FACTOR_ANCHO_SUP_INF and dif >= DIF_MIN_ANCHO:
            score += 2
            razones.append("sup>inf")

        if dif >= DIF_MIN_SUP_MAYOR_INF:
            score += 1
            razones.append("dif_ok")

        if abs(dif) <= TOLERANCIA_ANCHO_PARECIDO:
            score -= 1
            razones.append("parecidos")

    if det_sup is not None and det_mid is not None:
        if det_sup["ancho"] > det_mid["ancho"] + 8:
            score += 1
            razones.append("sup>mid")

    if det_inf is None and (det_mid is not None or det_sup is not None):
        score += 1
        razones.append("inf_none")

    return score, ",".join(razones) if razones else "sin_evidencia"


def verde_antes_de_interseccion(score):
    return score >= 2


def actualizar_permiso_verde(linea_detectada, det_inf, angle, error_x, historial):
    ok = (
        linea_detectada and
        det_inf is not None and
        abs(angle) <= ANGULO_MAX_VERDE_RECTO and
        abs(error_x) <= ERRX_MAX_VERDE_RECTO
    )

    historial.append(ok)
    if len(historial) > FRAMES_RECTO_PARA_VERDE:
        historial.pop(0)

    return len(historial) == FRAMES_RECTO_PARA_VERDE and all(historial)


def listo_para_ejecutar_verde(score, det_mid, det_inf, cy_ctrl, h, puede_leer_verde):
    if not puede_leer_verde:
        return False

    if not verde_antes_de_interseccion(score):
        return False

    if det_inf is None:
        return True

    if cy_ctrl is not None and cy_ctrl >= int(h * 0.60):
        return True

    if det_mid is not None and det_mid["cy_global"] >= int(h * 0.48):
        return True

    return False


def accion_estable_desde_historial(historial, n_frames):
    if len(historial) < n_frames:
        return None
    ultimos = historial[-n_frames:]
    if any(a is None for a in ultimos):
        return None
    if all(a == ultimos[0] for a in ultimos):
        return ultimos[0]
    return None


def resolver_accion_verde_pausa(historial):
    validas = [a for a in historial if a is not None]
    if not validas:
        return None

    conteo = {}
    for a in validas:
        conteo[a] = conteo.get(a, 0) + 1

    if conteo.get("U", 0) >= 2:
        return "U"
    return max(conteo, key=conteo.get)


def linea_totalmente_centrada(linea_detectada, error_x, angle, det_inf):
    if not linea_detectada or det_inf is None:
        return False
    return abs(error_x) <= CENTRAR_ERR_MAX and abs(angle) <= CENTRAR_ANG_MAX


def correccion_centrado(accion_post_giro, error_x, angle, linea_detectada):
    if linea_detectada:
        if error_x > CENTRAR_ERR_MAX:
            girar_derecha(VEL_CENTRAR_GIRO)
            return
        if error_x < -CENTRAR_ERR_MAX:
            girar_izquierda(VEL_CENTRAR_GIRO)
            return

        if angle > CENTRAR_ANG_MAX:
            girar_derecha(VEL_CENTRAR_MICRO)
            return
        if angle < -CENTRAR_ANG_MAX:
            girar_izquierda(VEL_CENTRAR_MICRO)
            return

        avanzar(VEL_CENTRAR_AVANCE)
        return

    if accion_post_giro == "IZQ":
        girar_izquierda(VEL_CENTRAR_GIRO)
    elif accion_post_giro == "DER":
        girar_derecha(VEL_CENTRAR_GIRO)
    elif accion_post_giro == "U":
        girar_derecha(VEL_CENTRAR_GIRO)
    else:
        stop_motores()


def correccion_sobre_eje(accion_post_giro, error_x, angle, linea_detectada):
    if linea_detectada:
        if error_x > EJE_ERR_OK or angle > EJE_ANG_OK:
            girar_derecha(VEL_EJE_GIRO)
            return
        if error_x < -EJE_ERR_OK or angle < -EJE_ANG_OK:
            girar_izquierda(VEL_EJE_GIRO)
            return
        stop_motores()
        return

    if accion_post_giro == "IZQ":
        girar_izquierda(VEL_EJE_GIRO)
    elif accion_post_giro == "DER":
        girar_derecha(VEL_EJE_GIRO)
    else:
        stop_motores()


def posible_gap(det_inf, historial_perdida):
    gap_now = (det_inf is None)

    historial_perdida.append(gap_now)

    if len(historial_perdida) > GAP_CONFIRM_FRAMES:
        historial_perdida.pop(0)

    if len(historial_perdida) < GAP_CONFIRM_FRAMES:
        return False

    return all(historial_perdida)


def detectar_curva_cerrada(linea_detectada, det_inf, det_mid, angle, error_x):
    if not linea_detectada:
        return False

    if det_inf is None and abs(angle) >= CURVA_REV_ANG_ON:
        return True

    if det_inf is None and det_mid is not None and abs(error_x) >= 28 and abs(angle) >= 28:
        return True

    if abs(angle) >= 52:
        return True

    return False


def seguir_linea(error_x, angle, cx_frame):
    vel = np.interp(abs(angle), [0, ANG_90], [VEL_BASE_MAX, VEL_BASE_MIN])
    corr = np.clip((angle * Kp_ANG) + (error_x / max(cx_frame, 1)) * Kp_ERR,
                   -MAX_CORR, MAX_CORR)
    vL = np.clip(vel - corr, -1.0, 1.0)
    vR = np.clip(vel + corr, -1.0, 1.0)
    motores(vL, vR, vL, vR)


def seguir_linea_post(error_x, angle, cx_frame):
    vel = np.interp(abs(angle), [0, ANG_90], [VEL_POST_MAX, VEL_POST_MIN])
    corr = np.clip((angle * (Kp_ANG * 1.10)) + (error_x / max(cx_frame, 1)) * (Kp_ERR * 1.20),
                   -0.22, 0.22)
    vL = np.clip(vel - corr, -1.0, 1.0)
    vR = np.clip(vel + corr, -1.0, 1.0)
    motores(vL, vR, vL, vR)


def seguir_linea_prep_verde(error_x, angle, cx_frame):
    vel = np.interp(abs(angle), [0, ANG_90], [VEL_PREP_VERDE_MAX, VEL_PREP_VERDE_MIN])
    corr = np.clip((angle * (Kp_ANG * 1.20)) + (error_x / max(cx_frame, 1)) * (Kp_ERR * 1.35),
                   -0.24, 0.24)
    vL = np.clip(vel - corr, -1.0, 1.0)
    vR = np.clip(vel + corr, -1.0, 1.0)
    motores(vL, vR, vL, vR)


def verde_requiere_reposicion(accion_preview, accion_decision, info_decision, error_x, angle, det_inf):
    if accion_preview is None and accion_decision is None:
        return False

    if abs(error_x) >= VERDE_REQUIERE_REPOSICION_SI_ERRX:
        return True

    if abs(angle) >= VERDE_REQUIERE_REPOSICION_SI_ANG:
        return True

    if VERDE_REPOSICIONAR_SI_DET_INF_NONE and det_inf is None:
        return True

    if info_decision is not None:
        tipo = info_decision.get("tipo", "")
        if "prioridad_arriba" in tipo or "empate_area" in tipo:
            return True

    return False


def correccion_reposicion_verde(error_x, angle, linea_detectada):
    if linea_detectada:
        if error_x > REPO_VERDE_ERR_OK:
            girar_derecha(VEL_REPO_GIRO)
            return
        if error_x < -REPO_VERDE_ERR_OK:
            girar_izquierda(VEL_REPO_GIRO)
            return

        if angle > REPO_VERDE_ANG_OK:
            girar_derecha(VEL_REPO_MICRO)
            return
        if angle < -REPO_VERDE_ANG_OK:
            girar_izquierda(VEL_REPO_MICRO)
            return

        stop_motores()
        return

    retroceder(VEL_REPO_BACK)


def listo_para_releer_verde(linea_detectada, det_inf, error_x, angle, historial):
    ok = (
        linea_detectada and
        det_inf is not None and
        abs(error_x) <= REPO_VERDE_ERR_OK and
        abs(angle) <= REPO_VERDE_ANG_OK
    )

    historial.append(ok)
    if len(historial) > REPO_VERDE_FRAMES_OK:
        historial.pop(0)

    return len(historial) == REPO_VERDE_FRAMES_OK and all(historial)

# ============================================================
# ESTADOS
# ============================================================
estado = "LINEA"
tiempo_estado = time.time()

ultima_marca_verde = 0
accion_verde = None
accion_verde_pendiente = None
accion_verde_reposicion = None
tiempo_verde_pendiente = 0
u_confirmada = False

ultimo_giro_normal = 0
pivot_mode = False
ultimo_sentido_linea = 1

verde_preview_hist = []
centrado_hist = []
gap_hist = []
post_giro_hist = []
recto_para_verde_hist = []
prep_verde_hist = []
curva_rev_hist = []
decision_verde_hist = []
eje_hist = []
repo_verde_hist = []

accion_post_giro = None
tiempo_inicio_centrado = 0.0
bloqueo_verde_hasta = 0.0
puede_leer_verde = True

# ============================================================
# LOOP PRINCIPAL
# ============================================================
try:
    while True:
        frame = cam.capture_array()
        h, w, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        tiempo_actual = time.time()
        cx_frame = w // 2

        y1_sup = int(h * 0.08)
        y2_sup = int(h * 0.28)
        y1_mid = int(h * 0.35)
        y2_mid = int(h * 0.58)
        y1_inf = int(h * 0.62)
        y2_inf = h
        y1_verde = int(h * 0.24)
        y2_verde = int(h * 0.80)
        y1_rojo = int(h * 0.70)
        y2_rojo = h

        roi_sup = hsv[y1_sup:y2_sup, 0:w]
        roi_mid = hsv[y1_mid:y2_mid, 0:w]
        roi_inf = hsv[y1_inf:y2_inf, 0:w]
        roi_verde = hsv[y1_verde:y2_verde, 0:w]
        roi_rojo = hsv[y1_rojo:y2_rojo, 0:w]

        negro_low, negro_high = leer_hsv("Calibracion Negro")
        verde_low, verde_high = leer_hsv("Calibracion Verde")
        rojo_low1, rojo_high1, rojo_low2, rojo_high2 = leer_hsv_rojo()

        mask_sup = limpiar_mascara(cv2.inRange(roi_sup, negro_low, negro_high), 1, 2)
        mask_mid = limpiar_mascara(cv2.inRange(roi_mid, negro_low, negro_high), 1, 2)
        mask_inf = limpiar_mascara(cv2.inRange(roi_inf, negro_low, negro_high), 1, 2)
        mask_verde = limpiar_mascara(cv2.inRange(roi_verde, verde_low, verde_high), 1, 2)

        mask_rojo_1 = cv2.inRange(roi_rojo, rojo_low1, rojo_high1)
        mask_rojo_2 = cv2.inRange(roi_rojo, rojo_low2, rojo_high2)
        mask_rojo = cv2.bitwise_or(mask_rojo_1, mask_rojo_2)
        mask_rojo = limpiar_mascara(mask_rojo, 1, 2)

        cv2.rectangle(frame, (0, y1_sup), (w, y2_sup), (180, 180, 180), 1)
        cv2.rectangle(frame, (0, y1_mid), (w, y2_mid), (255, 255, 255), 1)
        cv2.rectangle(frame, (0, y1_inf), (w, y2_inf), (255, 255, 255), 1)
        cv2.rectangle(frame, (0, y1_verde), (w, y2_verde), (0, 255, 0), 1)
        cv2.rectangle(frame, (0, y1_rojo), (w, y2_rojo), (0, 0, 255), 1)
        cv2.line(frame, (cx_frame, 0), (cx_frame, h), (100, 100, 100), 1)

        det_sup = detectar_objeto_principal(mask_sup, y1_sup, MIN_AREA_LINEA)
        det_mid = detectar_objeto_principal(mask_mid, y1_mid, MIN_AREA_LINEA)
        det_inf = detectar_objeto_principal(mask_inf, y1_inf, MIN_AREA_LINEA)

        if det_sup:
            dibujar_det_linea(frame, det_sup, y1_sup, (180, 180, 180), "SUP")
        if det_mid:
            dibujar_det_linea(frame, det_mid, y1_mid, (255, 255, 255), "MID")
        if det_inf:
            dibujar_det_linea(frame, det_inf, y1_inf, (255, 255, 255), "INF")

        det_rojo = detectar_objeto_principal(mask_rojo, y1_rojo, MIN_AREA_ROJO)
        if det_rojo is not None:
            x = det_rojo["x"]
            y = det_rojo["y"]
            w_rojo = det_rojo["w"]
            h_rojo = det_rojo["h"]
            cv2.rectangle(frame, (x, y), (x + w_rojo, y + h_rojo), (0, 0, 255), 2)
            cv2.circle(frame, (det_rojo["cx_global"], det_rojo["cy_global"]), 5, (0, 0, 255), -1)
            cv2.putText(frame, "ROJO STOP", (x, max(y - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
            cv2.putText(frame, "ROJO DETECTADO - STOP TOTAL", (10, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 255), 2)
            stop_motores()
            cv2.imshow("Robot", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            cv2.imshow("Mascara Negro SUP", mask_sup)
            cv2.imshow("Mascara Negro MID", mask_mid)
            cv2.imshow("Mascara Negro INF", mask_inf)
            cv2.imshow("Mascara Verde", mask_verde)
            cv2.imshow("Mascara Rojo", mask_rojo)
            cv2.waitKey(300)
            break

        ancho_sup = det_sup["ancho"] if det_sup else None
        ancho_mid = det_mid["ancho"] if det_mid else None
        ancho_inf = det_inf["ancho"] if det_inf else None

        linea_detectada = False
        cx_ctrl = None
        cy_ctrl = None
        angle = 0.0
        error_x = 0

        puntos = []
        if det_inf is not None:
            puntos.append((det_inf["cx_global"], det_inf["cy_global"], 0.58))
        if det_mid is not None:
            puntos.append((det_mid["cx_global"], det_mid["cy_global"], 0.28))
        if det_sup is not None:
            puntos.append((det_sup["cx_global"], det_sup["cy_global"], 0.14))

        if puntos:
            suma_pesos = sum(p[2] for p in puntos)
            cx_ctrl = int(sum(p[0] * p[2] for p in puntos) / suma_pesos)
            cy_ctrl = int(sum(p[1] * p[2] for p in puntos) / suma_pesos)
            linea_detectada = True

            dx = cx_ctrl - cx_frame
            dy = h - cy_ctrl
            if dy <= 0:
                dy = 1

            angle = math.degrees(math.atan2(dx, dy))
            error_x = cx_ctrl - cx_frame

            if abs(error_x) > 6:
                ultimo_sentido_linea = 1 if error_x > 0 else -1
            elif abs(angle) > 4:
                ultimo_sentido_linea = 1 if angle > 0 else -1

            cv2.circle(frame, (cx_ctrl, cy_ctrl), 6, (0, 255, 255), -1)
            cv2.line(frame, (cx_frame, h), (cx_ctrl, cy_ctrl), (0, 255, 255), 2)
        else:
            if det_mid is not None:
                error_x = det_mid["cx_global"] - cx_frame
            elif det_sup is not None:
                error_x = det_sup["cx_global"] - cx_frame

        detecciones_verde = detectar_verdes(mask_verde, y1_verde, MIN_AREA_VERDE)
        cx_ref = det_mid["cx_global"] if det_mid is not None else (cx_ctrl if cx_ctrl is not None else cx_frame)

        candidatos_preview = seleccionar_verdes_candidatos(
            detecciones_verde, cx_ref, y1_verde, y2_verde,
            FRACCION_Y_PREVIEW_MIN, FRACCION_Y_PREVIEW_MAX
        )

        candidatos_decision = seleccionar_verdes_candidatos(
            detecciones_verde, cx_ref, y1_verde, y2_verde,
            FRACCION_Y_DECISION_MIN, FRACCION_Y_DECISION_MAX
        )

        accion_preview, _ = decidir_accion_verde(candidatos_preview, cx_ref)
        accion_decision, info_decision = decidir_accion_verde(candidatos_decision, cx_ref)

        for c in candidatos_preview:
            dibujar_det_verde(frame, c["det"], f'P-{c["lado"]}')
        for c in candidatos_decision:
            dibujar_det_verde(frame, c["det"], f'D-{c["lado"]}')

        score_cruce, razon_cruce = score_interseccion(ancho_sup, ancho_mid, ancho_inf, det_sup, det_mid, det_inf)
        verde_habilitado = verde_antes_de_interseccion(score_cruce)

        verde_preview_hist.append(accion_preview if verde_habilitado and puede_leer_verde else None)
        if len(verde_preview_hist) > VERDE_CONFIRM_FRAMES:
            verde_preview_hist.pop(0)

        accion_preview_estable = accion_estable_desde_historial(verde_preview_hist, VERDE_CONFIRM_FRAMES)

        if accion_preview_estable is not None:
            accion_verde_pendiente = accion_preview_estable
            tiempo_verde_pendiente = tiempo_actual
            if accion_preview_estable == "U":
                u_confirmada = True

        if (tiempo_actual - tiempo_verde_pendiente) > TIMEOUT_VERDE_PENDIENTE:
            accion_verde_pendiente = None

        texto_verde = accion_verde_texto(
            (accion_decision if verde_habilitado and puede_leer_verde else None) or accion_verde_pendiente
        )

        cv2.putText(frame, f"Ang: {angle:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(frame, texto_verde, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
        cv2.putText(frame, f"CruceScore: {score_cruce} [{razon_cruce}]", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
        cv2.putText(frame, f"LeerVerde: {puede_leer_verde}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 0) if puede_leer_verde else (0, 0, 255), 2)
        cv2.putText(frame, f"U lock: {u_confirmada}", (10, 122),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                    (0, 255, 255) if u_confirmada else (120, 120, 120), 2)

        if info_decision is not None:
            cv2.putText(frame, f"Tipo: {info_decision['tipo']}", (10, 144),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 0), 2)

        # ====================================================
        # MÁQUINA DE ESTADOS
        # ====================================================
        if estado == "LINEA":
            permiso_por_tiempo = tiempo_actual >= bloqueo_verde_hasta
            permiso_por_rectitud = actualizar_permiso_verde(
                linea_detectada, det_inf, angle, error_x, recto_para_verde_hist
            )

            permiso_suave = (
                linea_detectada and
                det_inf is not None and
                abs(angle) <= 18 and
                abs(error_x) <= 40
            )

            puede_leer_verde = permiso_por_tiempo and (permiso_por_rectitud or permiso_suave)

            if not linea_detectada:
                recto_para_verde_hist.clear()

            en_curva_fuerte = abs(angle) >= 28 or (det_inf is None and det_mid is not None)

            if posible_gap(det_inf, gap_hist) and not en_curva_fuerte:
                estado = "GAP_FORWARD"
                tiempo_estado = tiempo_actual
                recto_para_verde_hist.clear()
                pivot_mode = False
            else:
                if det_inf is not None:
                    gap_hist.clear()

            if estado == "LINEA" and detectar_curva_cerrada(linea_detectada, det_inf, det_mid, angle, error_x):
                estado = "CURVA_CERRADA_REVERSA"
                tiempo_estado = tiempo_actual
                curva_rev_hist.clear()
                pivot_mode = False
                recto_para_verde_hist.clear()

            entrar_pivot = (
                linea_detectada and (
                    abs(angle) >= PIVOT_ANG_ON or
                    abs(error_x) >= PIVOT_ERR_ON or
                    det_inf is None
                )
            )

            salir_pivot = (
                linea_detectada and
                abs(angle) <= PIVOT_ANG_OFF and
                abs(error_x) <= PIVOT_ERR_OFF and
                det_inf is not None
            )

            if not pivot_mode and entrar_pivot:
                pivot_mode = True
            elif pivot_mode and salir_pivot:
                pivot_mode = False

            if estado == "LINEA":
                if pivot_mode:
                    v_pivot = velocidad_pivot(angle, error_x)
                    if angle > 0 or error_x > 0:
                        girar_derecha(v_pivot)
                    else:
                        girar_izquierda(v_pivot)
                else:
                    if linea_detectada:
                        seguir_linea(error_x, angle, cx_frame)
                    else:
                        stop_motores()

            if estado == "LINEA" and (tiempo_actual - ultima_marca_verde) > COOLDOWN_VERDE:
                if (accion_verde_pendiente is not None or u_confirmada) and verde_habilitado and permiso_por_tiempo:
                    necesita_repo = verde_requiere_reposicion(
                        accion_preview, accion_decision, info_decision,
                        error_x, angle, det_inf
                    )

                    if necesita_repo:
                        estado = "PREPARAR_VERDE_REVERSA"
                        tiempo_estado = tiempo_actual
                        repo_verde_hist.clear()
                        decision_verde_hist.clear()
                        pivot_mode = False
                        accion_verde_reposicion = "U" if u_confirmada else (accion_decision or accion_verde_pendiente)
                    else:
                        estado = "PREPARAR_VERDE"
                        tiempo_estado = tiempo_actual
                        prep_verde_hist.clear()
                        pivot_mode = False

            if estado == "LINEA":
                if (tiempo_actual - ultimo_giro_normal) > COOLDOWN_GIRO_NORMAL:
                    if (not pivot_mode) and det_inf is None and linea_detectada:
                        if angle <= -UMBRAL_ANG_GIRO_NORMAL:
                            estado = "GIRO_NORMAL_IZQ"
                            tiempo_estado = tiempo_actual
                            ultimo_giro_normal = tiempo_actual
                            recto_para_verde_hist.clear()
                        elif angle >= UMBRAL_ANG_GIRO_NORMAL:
                            estado = "GIRO_NORMAL_DER"
                            tiempo_estado = tiempo_actual
                            ultimo_giro_normal = tiempo_actual
                            recto_para_verde_hist.clear()

        elif estado == "PREPARAR_VERDE":
            if linea_detectada:
                seguir_linea_prep_verde(error_x, angle, cx_frame)

                ok_prep = (
                    abs(angle) <= PREP_VERDE_ANG_OK and
                    abs(error_x) <= PREP_VERDE_ERR_OK and
                    det_inf is not None
                )
                prep_verde_hist.append(ok_prep)
                if len(prep_verde_hist) > PREP_VERDE_FRAMES_OK:
                    prep_verde_hist.pop(0)
            else:
                stop_motores()

            if (tiempo_actual - ultima_marca_verde) > COOLDOWN_VERDE:
                if listo_para_ejecutar_verde(score_cruce, det_mid, det_inf, cy_ctrl, h, True):
                    accion_confirmada = "U" if u_confirmada else (accion_decision or accion_verde_pendiente)
                    if accion_confirmada is not None:
                        accion_verde = accion_confirmada
                        estado = "PAUSA_DECISION_VERDE"
                        tiempo_estado = tiempo_actual
                        ultima_marca_verde = tiempo_actual
                        accion_verde_pendiente = None
                        verde_preview_hist.clear()
                        recto_para_verde_hist.clear()
                        prep_verde_hist.clear()
                        decision_verde_hist.clear()

            if len(prep_verde_hist) == PREP_VERDE_FRAMES_OK and all(prep_verde_hist):
                accion_confirmada = "U" if u_confirmada else (accion_decision or accion_verde_pendiente)
                if accion_confirmada is not None and verde_habilitado:
                    accion_verde = accion_confirmada
                    estado = "PAUSA_DECISION_VERDE"
                    tiempo_estado = tiempo_actual
                    ultima_marca_verde = tiempo_actual
                    accion_verde_pendiente = None
                    verde_preview_hist.clear()
                    recto_para_verde_hist.clear()
                    prep_verde_hist.clear()
                    decision_verde_hist.clear()

            if (tiempo_actual - tiempo_estado) > PREP_VERDE_TIMEOUT:
                accion_confirmada = "U" if u_confirmada else (accion_decision or accion_verde_pendiente)
                if accion_confirmada is not None and verde_habilitado:
                    accion_verde = accion_confirmada
                    estado = "PAUSA_DECISION_VERDE"
                    tiempo_estado = tiempo_actual
                    ultima_marca_verde = tiempo_actual
                    accion_verde_pendiente = None
                    verde_preview_hist.clear()
                    recto_para_verde_hist.clear()
                    prep_verde_hist.clear()
                    decision_verde_hist.clear()
                else:
                    estado = "LINEA"
                    tiempo_estado = tiempo_actual
                    prep_verde_hist.clear()
                    u_confirmada = False

        elif estado == "PREPARAR_VERDE_REVERSA":
            retroceder(VEL_PREP_VERDE_REV)

            if (tiempo_actual - tiempo_estado) >= TIME_PREP_VERDE_REV:
                estado = "REPOSICIONAR_PARA_VERDE"
                tiempo_estado = tiempo_actual
                repo_verde_hist.clear()
                decision_verde_hist.clear()

        elif estado == "REPOSICIONAR_PARA_VERDE":
            correccion_reposicion_verde(error_x, angle, linea_detectada)

            listo = listo_para_releer_verde(
                linea_detectada, det_inf, error_x, angle, repo_verde_hist
            )

            if not u_confirmada:
                accion_actual_repo, _ = decidir_accion_verde(candidatos_decision, cx_ref)
                decision_verde_hist.append(accion_actual_repo)

                if len(decision_verde_hist) > PAUSA_VERDE_FRAMES_CONFIRM:
                    decision_verde_hist.pop(0)

                if len(decision_verde_hist) >= 3:
                    accion_estable = resolver_accion_verde_pausa(decision_verde_hist)
                    if accion_estable is not None:
                        accion_verde_reposicion = accion_estable
                        if accion_estable == "U":
                            u_confirmada = True
            else:
                accion_verde_reposicion = "U"

            if listo:
                accion_verde = "U" if u_confirmada else (accion_verde_reposicion or accion_decision or accion_verde_pendiente)
                estado = "PAUSA_DECISION_VERDE"
                tiempo_estado = tiempo_actual
                ultima_marca_verde = tiempo_actual
                accion_verde_pendiente = None
                verde_preview_hist.clear()
                recto_para_verde_hist.clear()
                repo_verde_hist.clear()
                decision_verde_hist.clear()

            elif (tiempo_actual - tiempo_estado) >= TIMEOUT_REPOSICION_VERDE:
                accion_verde = "U" if u_confirmada else (accion_verde_reposicion or accion_decision or accion_verde_pendiente)

                if accion_verde is not None:
                    estado = "PAUSA_DECISION_VERDE"
                    tiempo_estado = tiempo_actual
                    ultima_marca_verde = tiempo_actual
                    accion_verde_pendiente = None
                    verde_preview_hist.clear()
                    recto_para_verde_hist.clear()
                    repo_verde_hist.clear()
                    decision_verde_hist.clear()
                else:
                    estado = "LINEA"
                    tiempo_estado = tiempo_actual
                    repo_verde_hist.clear()
                    decision_verde_hist.clear()
                    u_confirmada = False

        elif estado == "PAUSA_DECISION_VERDE":
            stop_motores()

            if not u_confirmada:
                accion_actual_pausa, _ = decidir_accion_verde(candidatos_decision, cx_ref)
                decision_verde_hist.append(accion_actual_pausa)

                if len(decision_verde_hist) > PAUSA_VERDE_FRAMES_CONFIRM:
                    decision_verde_hist.pop(0)

                if len(decision_verde_hist) >= 3:
                    accion_estable = resolver_accion_verde_pausa(decision_verde_hist)
                    if accion_estable is not None:
                        accion_verde = accion_estable
                        if accion_estable == "U":
                            u_confirmada = True
            else:
                accion_verde = "U"

            if (tiempo_actual - tiempo_estado) >= PAUSA_VERDE_TIEMPO:
                if accion_verde is None:
                    accion_verde = "U" if u_confirmada else (accion_verde_reposicion or accion_decision or accion_verde_pendiente)

                estado = "RETROCEDE_VERDE"
                tiempo_estado = tiempo_actual
                decision_verde_hist.clear()

        elif estado == "RETROCEDE_VERDE":
            retroceder(VEL_RETROCESO_VERDE)

            if (tiempo_actual - tiempo_estado) >= TIEMPO_RETROCESO_VERDE:
                estado = "RELECTURA_VERDE"
                tiempo_estado = tiempo_actual
                decision_verde_hist.clear()

        elif estado == "RELECTURA_VERDE":
            avanzar(VEL_RELECTURA_VERDE)

            if not u_confirmada:
                accion_actual_relectura, _ = decidir_accion_verde(candidatos_decision, cx_ref)
                decision_verde_hist.append(accion_actual_relectura)

                if len(decision_verde_hist) > PAUSA_VERDE_FRAMES_CONFIRM:
                    decision_verde_hist.pop(0)

                if len(decision_verde_hist) >= 3:
                    accion_estable = resolver_accion_verde_pausa(decision_verde_hist)
                    if accion_estable is not None:
                        accion_verde = accion_estable
                        if accion_estable == "U":
                            u_confirmada = True
            else:
                accion_verde = "U"

            if (tiempo_actual - tiempo_estado) >= TIEMPO_RELECTURA_VERDE:
                if accion_verde is None:
                    accion_verde = "U" if u_confirmada else (accion_verde_reposicion or accion_decision or accion_verde_pendiente)

                estado = "AVANZA_VERDE"
                tiempo_estado = tiempo_actual
                decision_verde_hist.clear()

        elif estado == "GAP_FORWARD":
            avanzar(VEL_GAP_FORWARD)

            if (tiempo_actual - tiempo_estado) >= GAP_FORWARD_TIME:
                stop_motores()

                if GAP_EXIT_TO_CENTER and linea_detectada:
                    estado = "CENTRAR_LINEA"
                    tiempo_estado = tiempo_actual
                    tiempo_inicio_centrado = tiempo_actual
                    centrado_hist.clear()
                    gap_hist.clear()
                    accion_post_giro = None
                else:
                    estado = "LINEA"
                    tiempo_estado = tiempo_actual
                    gap_hist.clear()
                    pivot_mode = False

        elif estado == "CURVA_CERRADA_REVERSA":
            if linea_detectada:
                ok_curva = (
                    det_inf is not None and
                    abs(angle) <= CURVA_REV_ANG_OK and
                    abs(error_x) <= CURVA_REV_ERR_OK
                )
                curva_rev_hist.append(ok_curva)
            else:
                curva_rev_hist.append(False)

            if len(curva_rev_hist) > CURVA_REV_FRAMES_OK:
                curva_rev_hist.pop(0)

            t_local = tiempo_actual - tiempo_estado
            ciclo = CURVA_REV_PIVOT_1_TIME + CURVA_REV_BACK_TIME + CURVA_REV_PIVOT_2_TIME
            fase = t_local % ciclo

            girar_derecha_curva = (angle > 0 or error_x > 0)

            if fase < CURVA_REV_PIVOT_1_TIME:
                if girar_derecha_curva:
                    giro_reversa_derecha(CURVA_REV_PIVOT_VEL, CURVA_REV_BACK_VEL)
                else:
                    giro_reversa_izquierda(CURVA_REV_PIVOT_VEL, CURVA_REV_BACK_VEL)

            elif fase < (CURVA_REV_PIVOT_1_TIME + CURVA_REV_BACK_TIME):
                retroceder(CURVA_REV_BACK_VEL)

            else:
                if girar_derecha_curva:
                    giro_reversa_derecha(CURVA_REV_PIVOT_VEL * 0.94, CURVA_REV_BACK_VEL)
                else:
                    giro_reversa_izquierda(CURVA_REV_PIVOT_VEL * 0.94, CURVA_REV_BACK_VEL)

            if len(curva_rev_hist) == CURVA_REV_FRAMES_OK and all(curva_rev_hist):
                estado = "CENTRAR_LINEA"
                tiempo_estado = tiempo_actual
                tiempo_inicio_centrado = tiempo_actual
                centrado_hist.clear()
                curva_rev_hist.clear()
                accion_post_giro = "DER" if girar_derecha_curva else "IZQ"
                bloqueo_verde_hasta = tiempo_actual + 0.55
                puede_leer_verde = False

            elif (tiempo_actual - tiempo_estado) >= CURVA_REV_TIMEOUT:
                estado = "CENTRAR_LINEA"
                tiempo_estado = tiempo_actual
                tiempo_inicio_centrado = tiempo_actual
                centrado_hist.clear()
                curva_rev_hist.clear()
                accion_post_giro = "DER" if girar_derecha_curva else "IZQ"
                bloqueo_verde_hasta = tiempo_actual + 0.55
                puede_leer_verde = False

        elif estado == "GIRO_NORMAL_IZQ":
            girar_izquierda(VEL_GIRO_NORMAL)
            if (tiempo_actual - tiempo_estado) >= TIEMPO_GIRO_NORMAL_IZQ:
                estado = "CENTRAR_EJE"
                tiempo_estado = tiempo_actual
                accion_post_giro = "IZQ"
                eje_hist.clear()
                recto_para_verde_hist.clear()
                bloqueo_verde_hasta = tiempo_actual + 0.55
                puede_leer_verde = False

        elif estado == "GIRO_NORMAL_DER":
            girar_derecha(VEL_GIRO_NORMAL)
            if (tiempo_actual - tiempo_estado) >= TIEMPO_GIRO_NORMAL_DER:
                estado = "CENTRAR_EJE"
                tiempo_estado = tiempo_actual
                accion_post_giro = "DER"
                eje_hist.clear()
                recto_para_verde_hist.clear()
                bloqueo_verde_hasta = tiempo_actual + 0.55
                puede_leer_verde = False

        elif estado == "CENTRAR_EJE":
            if linea_detectada and det_inf is not None and abs(angle) <= EJE_ANG_OK and abs(error_x) <= EJE_ERR_OK:
                eje_hist.append(True)
            else:
                eje_hist.append(False)

            if len(eje_hist) > EJE_FRAMES_OK:
                eje_hist.pop(0)

            correccion_sobre_eje(accion_post_giro, error_x, angle, linea_detectada)

            if len(eje_hist) == EJE_FRAMES_OK and all(eje_hist):
                estado = "LINEA"
                tiempo_estado = tiempo_actual
                eje_hist.clear()
                accion_post_giro = None
                pivot_mode = False
                puede_leer_verde = False

            elif (tiempo_actual - tiempo_estado) >= EJE_TIMEOUT:
                estado = "LINEA"
                tiempo_estado = tiempo_actual
                eje_hist.clear()
                accion_post_giro = None
                pivot_mode = False
                puede_leer_verde = False

        elif estado == "AVANZA_VERDE":
            if u_confirmada:
                accion_verde = "U"

            if accion_verde == "U":
                avanzar(VEL_AVANCE_U)
                if (tiempo_actual - tiempo_estado) >= TIEMPO_AVANCE_U:
                    estado = "GIRO_VERDE_U"
                    tiempo_estado = tiempo_actual
                    recto_para_verde_hist.clear()
            else:
                avanzar(VEL_AVANCE_VERDE)
                if (tiempo_actual - tiempo_estado) >= TIEMPO_AVANCE_VERDE:
                    if accion_verde == "IZQ":
                        estado = "GIRO_VERDE_IZQ"
                    elif accion_verde == "DER":
                        estado = "GIRO_VERDE_DER"
                    else:
                        estado = "LINEA"
                        u_confirmada = False
                    tiempo_estado = tiempo_actual
                    recto_para_verde_hist.clear()

        elif estado == "GIRO_VERDE_IZQ":
            girar_izquierda(VEL_GIRO_VERDE)
            if (tiempo_actual - tiempo_estado) >= TIEMPO_GIRO_VERDE_IZQ:
                estado = "POST_GIRO_ESTABILIZAR"
                tiempo_estado = tiempo_actual
                accion_post_giro = "IZQ"
                post_giro_hist.clear()
                recto_para_verde_hist.clear()
                bloqueo_verde_hasta = tiempo_actual + TIEMPO_BLOQUEO_VERDE_POSTGIRO
                puede_leer_verde = False
                accion_verde_reposicion = None
                u_confirmada = False

        elif estado == "GIRO_VERDE_DER":
            girar_derecha(VEL_GIRO_VERDE)
            if (tiempo_actual - tiempo_estado) >= TIEMPO_GIRO_VERDE_DER:
                estado = "POST_GIRO_ESTABILIZAR"
                tiempo_estado = tiempo_actual
                accion_post_giro = "DER"
                post_giro_hist.clear()
                recto_para_verde_hist.clear()
                bloqueo_verde_hasta = tiempo_actual + TIEMPO_BLOQUEO_VERDE_POSTGIRO
                puede_leer_verde = False
                accion_verde_reposicion = None
                u_confirmada = False

        elif estado == "GIRO_VERDE_U":
            girar_derecha(VEL_GIRO_VERDE_U)
            if (tiempo_actual - tiempo_estado) >= TIEMPO_GIRO_VERDE_U:
                estado = "POST_GIRO_ESTABILIZAR"
                tiempo_estado = tiempo_actual
                accion_post_giro = "U"
                post_giro_hist.clear()
                recto_para_verde_hist.clear()
                bloqueo_verde_hasta = tiempo_actual + TIEMPO_BLOQUEO_VERDE_U
                puede_leer_verde = False
                accion_verde_reposicion = None
                u_confirmada = False

        elif estado == "POST_GIRO_ESTABILIZAR":
            if linea_detectada:
                seguir_linea_post(error_x, angle, cx_frame)

                ok = (
                    abs(angle) <= POST_GIRO_ANG_OK and
                    abs(error_x) <= POST_GIRO_ERR_OK and
                    det_inf is not None
                )

                post_giro_hist.append(ok)
                if len(post_giro_hist) > POST_GIRO_FRAMES_OK:
                    post_giro_hist.pop(0)

                if len(post_giro_hist) == POST_GIRO_FRAMES_OK and all(post_giro_hist):
                    estado = "LINEA"
                    tiempo_estado = tiempo_actual
                    accion_verde = None
                    pivot_mode = False
                    puede_leer_verde = False
                    u_confirmada = False
            else:
                correccion_centrado(accion_post_giro, error_x, angle, linea_detectada)

            if (tiempo_actual - tiempo_estado) > POST_GIRO_TIMEOUT:
                estado = "CENTRAR_LINEA"
                tiempo_estado = tiempo_actual
                tiempo_inicio_centrado = tiempo_actual
                centrado_hist.clear()

        elif estado == "CENTRAR_LINEA":
            if linea_totalmente_centrada(linea_detectada, error_x, angle, det_inf):
                centrado_hist.append(True)
            else:
                centrado_hist.append(False)

            if len(centrado_hist) > CENTRAR_FRAMES_OK:
                centrado_hist.pop(0)

            correccion_centrado(accion_post_giro, error_x, angle, linea_detectada)

            if len(centrado_hist) == CENTRAR_FRAMES_OK and all(centrado_hist):
                estado = "LINEA"
                tiempo_estado = tiempo_actual
                centrado_hist.clear()
                accion_post_giro = None
                accion_verde = None
                pivot_mode = False
                puede_leer_verde = False
                u_confirmada = False

            elif (tiempo_actual - tiempo_inicio_centrado) > TIMEOUT_CENTRADO:
                if linea_detectada:
                    estado = "LINEA"
                    tiempo_estado = tiempo_actual
                    centrado_hist.clear()
                    accion_post_giro = None
                    accion_verde = None
                    pivot_mode = False
                    puede_leer_verde = False
                    u_confirmada = False
                else:
                    stop_motores()

        cv2.putText(frame, f"Estado: {estado}", (10, h - 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.putText(frame, f"ErrX: {error_x}  Ang: {angle:.1f}", (10, h - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        cv2.imshow("Robot", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.imshow("Mascara Negro SUP", mask_sup)
        cv2.imshow("Mascara Negro MID", mask_mid)
        cv2.imshow("Mascara Negro INF", mask_inf)
        cv2.imshow("Mascara Verde", mask_verde)
        cv2.imshow("Mascara Rojo", mask_rojo)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    stop_motores()
    cv2.destroyAllWindows()
    try:
        cam.stop()
    except Exception:
        pass
