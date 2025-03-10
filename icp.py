# ICP
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
