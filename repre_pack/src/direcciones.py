#gesture   yolo

texto = """arriba-derecha derecha 0
arriba-derecha izquierda 2
arriba-derecha centro 0
arriba derecha 0
arriba izquierda 1
arriba centro 2
arriba-izquierda derecha 2
arriba-izquierda izquierda 1
arriba-izquierda centro 1
derecha derecha 5
derecha izquierda 4
derecha centro 5
centro derecha 5
centro izquierda 3
centro centro 4
izquierda derecha 4
izquierda izquierda 3
izquierda centro 3
NONE derecha 7
NONE izquierda 7
NONE centro 7
Izquierda-atras derecha 6
Izquierda-atras izquierda 6
Izquierda-atras centro 6
"""

columna_1 = []
columna_2 = []
columna_3 = []

lineas = texto.split('\n')
for linea in lineas:
    valores = linea.split()
    if len(valores) >= 3:
        columna_1.append(valores[0])
        columna_2.append(valores[1])
        columna_3.append(valores[2])

# Definir una función para obtener la salida dado dato_1 y dato_2
def obtener_salida(gesture, yolo):
    indice_combinacion = -1  # Inicializamos con un valor que indica que no se encontró ninguna combinación

    print("manitas :" + gesture)
    print("donde toy ? :" + yolo)
    
    for i in range(len(columna_1)):
        if columna_1[i] == gesture and columna_2[i] == yolo:
            indice_combinacion = i
            break

    if indice_combinacion != -1:
        return columna_3[indice_combinacion]
    else:
        return None
