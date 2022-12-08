import numpy as np


mu0 = np.pi * 4 * 10 * -7

def _getStartV(start_angels, v0):
    v = np.array([np.cos(start_angels[0]), np.sin(start_angels[0]), np.sin(start_angels[1])])
    return np.array(v0 * v / np.linalg.norm(v)).reshape(-1, 3)


def _getModuleB(I, point):
    return mu0 / (4 * np.pi) * 2 * I / np.linalg.norm(point[1:])


def _getB(I, point):
    # B = _getModuleB(I, point)  # модуль вектора В
    r = np.array([0, point[1], point[2]])  # вектор от провода до заряда
    vectB = np.cross([1, 0, 0], r)  # направление магнитного поля
    normVectB = vectB / np.linalg.norm(vectB)  # нормированное направление вектор B
    return normVectB * _getModuleB(I, point)  # направление на модуль - искомый вектор


def _getF(q, V, I, point):
    return q * np.cross(V, _getB(I, point))


def _calculate_speed(V, q, m, dt, I, point):
    return V + _getF(q, V, I, point)/m * dt


def _calculate_position(point, V, dt):
    return point + V * dt


def calculate_trajectory(start_position, start_angels, v0, q, m, I, R, t_max, dt=0.001):
    t = 0
    V = _getStartV(start_angels, v0)
    positions = np.array(start_position).reshape(-1, 3)
    while np.linalg.norm(positions[-1][1:]) > R and t < t_max:
        t += dt
        V = np.append(V, _calculate_speed(V[-1], q, m, dt, I, positions[-1]).reshape(1,3), axis=0)
        positions = np.append(positions, _calculate_position(positions[-1], V[-1], dt).reshape(1,3), axis=0)
    return positions
