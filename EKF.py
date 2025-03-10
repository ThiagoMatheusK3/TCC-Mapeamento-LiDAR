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
