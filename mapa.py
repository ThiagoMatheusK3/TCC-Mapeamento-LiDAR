def local_global (x_local, y_local, carro_x, carro_y, carro_theta):
    #|x_global|   | cos  -sin  x_carro |   |x_local|
    #|y_global| = | sin   cos  y_carro | * |y_local| 
    #|    1   |   |  0     0      1    |   |   1   |
    x_global = x_local * np.cos(carro_theta) - y_local * np.sin(carro_theta) + carro_x
    y_global = x_local * np.sin(carro_theta) + y_local * np.cos(carro_theta) + carro_y
    return x_global, y_global

def log_ods_conversao(log_odds):
    return 1 - (1 / (1 + np.exp(log_odds)))

# Converte o mapa de Log-Odds para probabilidade
    mapa_probabilidade = log_ods_conversao(mapa_grade)
