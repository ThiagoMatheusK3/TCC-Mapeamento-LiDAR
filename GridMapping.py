### GRID MAPPING ###
tamanho_mapa = 4000  
tamanho_celula = 10 
resolucao_grid = int(tamanho_mapa / tamanho_celula)
mapa_grade = np.zeros((resolucao_grid, resolucao_grid))

def indice_grid(x, y, tamanho_celula):
    x_central = x + (tamanho_mapa / 2)
    y_central = y + (tamanho_mapa / 2)
    i = int(y_central / tamanho_celula)
    j = int(x_central / tamanho_celula)
    return i, j

