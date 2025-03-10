# Código usado na coleta de dadados dia 27/02/2025
# 

import numpy as np
import math
import matplotlib.pyplot as plt
import open3d as o3d
from matplotlib.animation import FuncAnimation
import socket
import threading
from bresenham import bresenham
from pynput import keyboard
import time
import csv


### COMUNICAÇÃO ###
HOST = "192.168.120.102"
PORTA = 80
buffer = ""


### GRID MAPPING ###
tamanho_mapa = 4000  
tamanho_celula = 40 
resolucao_grid = int(tamanho_mapa / tamanho_celula)
mapa_grade = np.zeros((resolucao_grid, resolucao_grid))

trajetoria_prevista = []
trajetoria_corrigida = []
dados_lidar = []
trajetoria_odometria = []

pontos_nuvem = []
ocupacao_anterior = None  # nuvem t-1
odometria_acumulada = np.eye(4)  # Inicializa a odometria com matriz identidade


nova_medicao_icp = False  # Flag para indicar quando o ICP tem novos dados
vetor_observacao = np.array([[0], [0], [0]])  # Inicializa deslocamento do ICP como zero


def obter_entrada_controle(): 
    global controle_ativo, u
    if controle_ativo:
        controle_ativo = False  # Reset da Flag
        return u
    return np.array([0, 0]) 



### FUNÇÕES ###

# TRANSFORMAÇÕES
def indice_grid(x, y, tamanho_celula):
    x_central = x + (tamanho_mapa / 2)
    y_central = y + (tamanho_mapa / 2)
    i = int(y_central / tamanho_celula)
    j = int(x_central / tamanho_celula)
    return i, j


def graus_radianos(graus):
    return math.radians(graus)

def polar_cartesiano(r, theta):
    x_local = r * np.cos(theta)
    y_local = r * np.sin(theta)
    return x_local, y_local

def local_global (x_local, y_local, carro_x, carro_y, carro_theta):

    #|x_global|   | cos  -sin  x_carro |   |x_local|
    #|y_global| = | sin   cos  y_carro | * |y_local| 
    #|    1   |   |  0     0      1    |   |   1   |


    x_global = x_local * np.cos(carro_theta) - y_local * np.sin(carro_theta) + carro_x
    y_global = x_local * np.sin(carro_theta) + y_local * np.cos(carro_theta) + carro_y
    
    return x_global, y_global




# LOG ODDS
def log_odds (valor_atual, leitura, ocupacao=0.60, livre=0.40): ##OBS: Confiança a respeito do objeto

    if leitura == 1:
        incremento = math.log(ocupacao/livre)

    else:
        incremento = math.log(livre/ocupacao)
        
    novo_valor = valor_atual + incremento
    return max(-10,min(10, novo_valor)) ##Normalização - nao crescer indefinidamente

def log_ods_conversao(log_odds):
    return 1 - (1 / (1 + np.exp(log_odds)))


def atenuacao(log_odds, taxa=0.02): ##TESTE
    if log_odds > 0.0:
        return max(0.0, log_odds - taxa)
    elif log_odds < 0.0:
        return min(0.0, log_odds + taxa)
    return log_odds



#### ICP ###

def conversao_2D_3D(pontos_2D):
    
    pcd = o3d.geometry.PointCloud()
    pontos_3D = np.hstack((pontos_2D, np.zeros((pontos_2D.shape[0], 1))))  # Adiciona z=0
    pcd.points = o3d.utility.Vector3dVector(pontos_3D)
    
    return pcd

def nuvem_icp (fonte_pontos, referencia_pontos, distancia=40, init=np.eye(4)):

    fonte_pcd = conversao_2D_3D(fonte_pontos)
    referencia_pcd = conversao_2D_3D(referencia_pontos)
    
    reg_result = o3d.pipelines.registration.registration_icp(
        fonte_pcd, referencia_pcd, distancia, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500)
    )
    return reg_result

def localizacao (transformation): ## Transformada proveniente do ICP
    
    #T = [ [cos(theta), -sin(theta), 0, tx],
    #      [sin(theta),  cos(theta), 0, ty],
    #      [   0,           0,       1,  0],
    #      [   0,           0,       0,  1] ]
    
    # Extrai a rotação usando a função arctan2: 
    theta = np.arctan2(transformation[1, 0], transformation[0, 0])

    # Extrai a translação:
    tx = transformation[0, 3]
    ty = transformation[1, 3]

    return tx, ty, theta





### PONTOS ###

def salvar_dados_lidar(x_local, y_local):
    timestamp = time.time()
    with open("dados_lidar.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, x_local, y_local])

#Função para salvar a trajetória prevista
def salvar_trajetoria_prevista(x, y, theta):
    timestamp = time.time()
    with open("trajetoria_prevista.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, x, y, theta])

#Função para salvar a trajetória corrigida
def salvar_trajetoria_corrigida(x, y, theta):
    timestamp = time.time()
    with open("trajetoria_corrigida.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, x, y, theta])


def salvar_trajetoria_odometria(x, y, theta):
    timestamp = time.time()
    with open("trajetoria_odometria.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, x, y, theta])




# FILTRO DE KALMAN ESTENDIDO

def predicao (x, P, u, Q, dt):
    v, w = u  # Velocidade linear e angular média, contabilizada pelo RPM
    theta = x[2, 0]  # Ângulo theta atual

    x_belif = x + np.array([[v * np.cos(theta) * dt],
                            [v * np.sin(theta) * dt],
                            [w * dt]])

    J = np.array([[1, 0, -v * np.sin(theta) * dt],
                  [0, 1,  v * np.cos(theta) * dt],
                  [0, 0, 1]])

    P_belif = J @ P @ J.T + Q
    return x_belif, P_belif

def atualizacao (x, P, z, H, R):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_atualizacao = x + K @ y
    P_atualizacao = (np.eye(len(x)) - K @ H) @ P
    return x_atualizacao, P_atualizacao

### VARIÁVEIS DO KALMAN ###
x = np.array([[0], [0], [0]])  # Posição inicial (x, y, theta)
P = np.diag([0.01, 0.01, 0.01]) # Convariancia 
Q = np.diag([1, 1, 0.001]) # Ruído do sistema
R = np.diag([400, 400, 0.001]) # Ruiído da observação





def ekf():
    global x, P, nova_medicao_icp, vetor_observacao, trajetoria_prevista, trajetoria_corrigida

    dt = 1  # Intervalo de tempo entre predições
    x_ideal = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
    

    while True:
        #Predição

        u = obter_entrada_controle() 

        v, w = u
        theta_ideal = x_ideal[2, 0]
        x_ideal += np.array([[v * np.cos(theta_ideal) * dt],
                             [v * np.sin(theta_ideal) * dt],
                             [w * dt]])


        trajetoria_prevista.append((x_ideal[0, 0], x_ideal[1, 0], x_ideal[2,0]))
        salvar_trajetoria_prevista(*trajetoria_prevista[-1])


        x, P = predicao(x, P, u, Q, dt)
        #print(f"EKF PREDIÇÃO: x = {x.ravel()}")

        #Correção 
        if nova_medicao_icp:

            z = np.array([[vetor_observacao[0, 2]],  # Translação X
              [vetor_observacao[1, 2]],  # Translação Y
              [carro_theta2]])  # theta IMU)

            H = np.eye(3)   

            x, P = atualizacao(x, P, z, H, R)
            #print(f"EKF CORREÇÃO: x = {x.ravel()}")


            trajetoria_corrigida.append((x[0, 0], x[1, 0], x[2, 0]))

            nova_medicao_icp = False  # Reseta a flag após usar os dados

        time.sleep(dt)  # Mantém a sincronização











def coleta_dados(sock):
    global buffer, mapa_grade, carro_x, carro_y, carro_theta, carro_theta2
    while True:
        try:
            data = sock.recv(2048)
            if not data:
                continue

            buffer += data.decode('utf-8')

            while '\n' in buffer:
                mensagem, buffer = buffer.split('\n', 1)

                try:
                    valores = mensagem.split(',')

                    if len(valores) == 3:
                        angulo, distancia, carro_theta = map(float, valores)

                        if distancia == 0: ##Teste Atenuação, quando o Lidar detecta "0"
                            mapa_grade = np.vectorize(atenuacao)(mapa_grade)
                            continue

                        theta = graus_radianos(angulo)
                        
                        x_local, y_local = polar_cartesiano(distancia, theta)

                        #salvar_dados_lidar(x_local, y_local)

                        pontos_nuvem.append([x_local, y_local]) ##pontos para ICP

                        carro_theta2 = graus_radianos(carro_theta)
                        
                        carro_x, carro_y = x[0,0], x[1,0]
                        carro_angulo = x[2,0]

                        x_global, y_global = local_global(x_local, y_local, carro_x, carro_y, carro_angulo) # manda pro referencial global.

                        

                        i_car, j_car = indice_grid(x_global, y_global, tamanho_celula)
                        i_sensor, j_sensor = indice_grid(carro_x, carro_y, tamanho_celula) # algoritimo de bresenham

                        pontos_linha = list(bresenham(i_sensor, j_sensor, i_car, j_car))

                        for i, j in pontos_linha[:-1]: #um ponto antes da célula
                            try:
                                mapa_grade[i, j] = log_odds(mapa_grade[i, j], leitura=0)
                            except IndexError:
                                pass

                        try:
                            mapa_grade[i_car, j_car] = log_odds(mapa_grade[i_car, j_car], leitura=1)
                        except IndexError:
                            print(f" fora dos limites: {x_global}, {y_global}")

                except ValueError:
                    print("Erro valores recebidos.")

        except ConnectionResetError:
            print("Falha comunicação ESP32.")
            break


def atualizar_coletas(): ## TESTE
    global pontos_nuvem, ocupacao_anterior, odometria_acumulada
    global nova_medicao_icp, vetor_observacao, trajetoria_odometria
    while True:
        
        if len(pontos_nuvem) >= 650:
            # Usa os últimos 650 pontos para formar a varredura atual
            pontos_atual = np.array(pontos_nuvem[-650:])
            
            if ocupacao_anterior is not None and ocupacao_anterior.size > 0:
                
                #start_time = time.time()  # Inicia a medição do tempo do ICP
                
                icp_result = nuvem_icp(pontos_atual, ocupacao_anterior)
                #tx, ty, theta = localizacao(icp_result.transformation)

                nova_medicao_icp = True

                #end_time = time.time()    # Termina a medição
                #processing_time = end_time - start_time

                odometria_acumulada = np.dot(odometria_acumulada, icp_result.transformation)
                vetor_observacao = odometria_acumulada

                #print(f"Fitness: {icp_result.fitness:.4f}")
                #print(f"Erro médio: {icp_result.inlier_rmse:.4f}")
                #print("DELTA: x = {:.5f}, y = {:.5f}, theta = {:.5f} rad".format(tx, ty, theta))
                #print(odometria_acumulada)
                #print("Tempo de processamento do ICP: {:.5f} segundos".format(processing_time))

            else:
                print("Primeira coleta: sem ICP")

 
            ocupacao_anterior = pontos_atual.copy()
            
            pontos_nuvem = pontos_nuvem[-650:]
        
        time.sleep(0.2)


# Variáveis globais
controle_ativo = False  # Flag para indicar se um comando foi enviado
u = np.array([0, 0])  # Vetor de entrada de controle [v, w]

def enviar_comandos(sock):
    global controle_ativo, u

    def on_press(key):
        global controle_ativo, u  
        try:

            v = 120 * (2 * np.pi / 60) * 35  # Convertendo RPM para mm/s
            w = np.radians(435.48)  # Velocidade angular fixa

            if key == keyboard.Key.up:
                u = np.array([v, 0])
                sock.sendall(b"1\n")  
            elif key == keyboard.Key.down:
                u = np.array([-v, 0])
                sock.sendall(b"2\n")  
            elif key == keyboard.Key.left:
                u = np.array([0, w])
                sock.sendall(b"3\n")  
            elif key == keyboard.Key.right:
                u = np.array([0, -w])
                sock.sendall(b"4\n")  
            elif key == keyboard.Key.esc:
                print("Encerrando controle.")
                u = np.array([0, 0])  
                sock.sendall(b"0\n")  # Comando de parada
                return False  # Encerra o listener
            else:
                return  # Ignora outras teclas

            # Flag
            controle_ativo = True

        except Exception as e:
            print(f"Erro ao enviar comando: {e}")

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()







def atualiza_mapa(frame):
    global mapa_grade, trajetoria_corrigida, trajetoria_prevista

    plt.clf()
    
    # Converte o mapa de Log-Odds para probabilidade
    mapa_probabilidade = log_ods_conversao(mapa_grade)

    plt.imshow(
        mapa_probabilidade,
        cmap="gray_r",
        origin="lower",
        extent=[-tamanho_mapa / 2, tamanho_mapa / 2, -tamanho_mapa / 2, tamanho_mapa / 2],
        vmin=0.0,
        vmax=1.0
    )
    plt.colorbar(label="Probabilidade de Ocupação")
    plt.title("Mapa")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")



    # Plota a trajetória ideal (modelo cinemático do monociclo)
    if len(trajetoria_prevista) > 1:
        ideal_x, ideal_y, ideal_theta = zip(*trajetoria_prevista)
        plt.plot(ideal_x, ideal_y, 'k--', label="Trajetória Ideal (Modelo Cinemático)")

    # Plota a trajetória corrigida pelo EKF
    if len(trajetoria_corrigida) > 1:
        corrigido_x, corrigido_y, corrigido_theta = zip(*trajetoria_corrigida)
        plt.plot(corrigido_x, corrigido_y, 'g--', label="Trajetória Corrigida (EKF)")

    # Plota a posição atual do robô (última posição da trajetória corrigida)
    if len(trajetoria_corrigida) > 0:
        carro_x, carro_y, carro_theta = trajetoria_corrigida[-1]
        plt.scatter(carro_x, carro_y, color='red', s=60, marker='o', label='Robô Atual')

    



if __name__ == "__main__":
    try:

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORTA))

        
        threading.Thread(target=ekf, daemon=True).start()
        threading.Thread(target=coleta_dados, args=(sock,), daemon=True).start()
        thread_atualizacao = threading.Thread(target=atualizar_coletas, daemon=True)
        threading.Thread(target=enviar_comandos, args=(sock,), daemon=True).start()
        thread_atualizacao.start()
        


        fig_dinamico = plt.figure("Mapa Dinâmico")
        ani_dinamico = FuncAnimation(fig_dinamico, atualiza_mapa, interval=33, cache_frame_data=False)
        plt.show()

        plt.pause(0.01)

    except KeyboardInterrupt:
        print("Interrompendo")
