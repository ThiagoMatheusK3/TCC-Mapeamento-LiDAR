# TRANSFORMAÇÕES
def polar_cartesiano(r, theta):
    x_local = r * np.cos(theta)
    y_local = r * np.sin(theta)
    return x_local, y_local
