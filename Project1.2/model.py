import numpy as np

def _calculate_speeds(x, y, V_x, V_y, m, Q, q, dt, epsilon):
    V_xn = V_x[-1] + dt * q * Q * x[-1] / ((4 * np.pi * epsilon * m *
                                           8.8542 * 10 ** -12) * (x[-1]**2 + y[-1]**2)**3/2)
    V_yn = V_y[-1] + dt * q * Q * y[-1] / ((4 * np.pi * epsilon * m *
                                            8.8542 * 10 ** -12) * (x[-1]**2 + y[-1]**2)**3/2)
    return V_xn, V_yn


def _calculate_locs(x, y, V_x, V_y, dt):
    x_n = x[-1] + dt * V_x[-1]
    y_n = y[-1] + dt * V_y[-1]
    return (x_n, y_n)


def calculate_trajectory(l=0, q=0, Q=0, m=0, alpha=0, V_0=0, dt=0, epsilon=1):
    x = np.array([l])
    y = np.array([0])
    V_x = np.array([V_0 * np.cos(alpha)])
    V_y = np.array([V_0 * np.sin(alpha)])
    t = 0
    i = 0
    while 0.01 <x[-1]**2 + y[-1]**2 < (3*l) ** 2 and t < 1:
        i += 1
        t += dt
        V_xn, V_yn = _calculate_speeds(x, y, V_x, V_y, m, Q, q, dt, epsilon)
        V_x = np.append(V_x, V_xn)
        V_y = np.append(V_y, V_yn)
        x_n, y_n = _calculate_locs(x, y, V_x, V_y, dt)
        x = np.append(x, x_n)
        y = np.append(y, y_n)
        # print(i , x[-1], y[-1])
    return (x, y)
