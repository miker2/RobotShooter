import math
import cmath
import numpy as np


# A basic 2nd order filtering class.
class Filter2ndOrder:
    def __init__(self, dt, freq_hz, gain=1, quality=math.sqrt(2.0) / 2.0):
        # Call the filter setup function:
        self.set_params(dt, freq_hz, gain, quality)

    def set_params(self, dt, freq_hz, gain=1.0, q=math.sqrt(2.0) / 2.0):
        if dt <= 0:
            raise ValueError("Value of 'dt' must be greater than 0.")

        self._dt = dt

        zeta = 1.0 / (2 * q)
        w0   = freq_hz * 2 * math.pi
        D    = zeta ** 2 - 1
        p    = (-w0 * (zeta + cmath.sqrt(D)), -w0 * (zeta - cmath.sqrt(D)))

        self._s_poles = np.array(p,  dtype=np.complex64)
        self._s_zeros = np.array([], dtype=np.complex64)
        self._s_n_poles = len(p)
        self._s_n_zeros = 0
        self._s_gain = gain * w0 ** 2

        self._z_poles = []
        self._z_zeros = []
        self._z_n_poles = 0
        self._z_n_zeros = 0
        self._z_gain = 0
        
        self._c2d()

        self._generate_diff_eqn()

        self._zi = np.array([], dtype=np.float32)

        self._cx = np.array([0.0, 0.0], dtype=np.float32)
        self._cy = np.array([0.0, 0.0], dtype=np.float32)
        self._cxn = self._B[2] / self._A[2]
        self._cx[0] = self._B[1] / self._A[2]
        self._cx[1] = self._B[0] / self._A[2]
        self._cy[0] = self._A[1] / self._A[2]
        self._cy[1] = self._A[0] / self._A[2]

    def filter_val(self, val):
        if not self._zi.size:
            self._init(val)

        out = val * self._cxn + self._zi[0]
        self._zi[0] = -out * self._cy[0] + val * self._cx[0] + self._zi[1]
        self._zi[1] = -out * self._cy[1] + val * self._cx[1]

        return out

    def _c2d(self):
        samp_freq = 1 / self._dt
        sample_time = self._dt

        for z in self._s_zeros:
            zero = z * sample_time / 2
            self._z_zeros.append((1 + zero) / (1 - zero))
        self._z_n_zeros = self._s_n_zeros

        for p in self._s_poles:
            pole = p * sample_time / 2
            self._z_poles.append((1 + pole) / (1 - pole))
        self._z_n_poles = self._s_n_poles

        g = complex(1, 0)
        for z in self._s_zeros:
            g *= 2 * samp_freq - z
        for p in self._s_poles:
            g /= 2 * samp_freq - p

        self._z_gain = (g * self._s_gain).real

        while self._z_n_zeros < self._z_n_poles:
            self._z_zeros.append(complex(-1, 0))
            self._z_n_zeros += 1

    def _generate_diff_eqn(self):
        self._B = self._z_gain * np.poly1d(self._z_zeros, True)
        self._A = np.poly1d(self._z_poles, True)

    def _init(self, val):
        y = val * (self._cxn + self._cx[0] + self._cx[1]) / (1.0 + self._cy[0] + self._cy[1])
        self._zi = np.array([0, 0], dtype=np.float32)
        self._zi[1] = -y * self._cy[1] + val * self._cx[1]
        self._zi[0] = -y * self._cy[0] + val * self._cx[0] + self._zi[1]


def rot2d(yaw, p):
    x_out = math.cos(yaw) * p[0] - math.sin(yaw) * p[1]
    y_out = math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
    return x_out, y_out


def deadband(val, a, b):
    min_val = min(a, b)
    max_val = max(a, b)
    if (min_val < val) and (val < max_val):
        return 0
    elif val < min_val:
        return val - min_val
    elif val > max_val:
        return val - max_val
    else:
        return 0


def saturate(val, _min, _max):
    if val < _min:
        return _min
    elif val > _max:
        return _max
    else:
        return val


def slew_rate_limit(cur, des, limit, dt):
    if dt <= 0 or limit < 0:
        return cur
    change = saturate((des - cur) / dt, -limit, limit) * dt
    return cur + change
