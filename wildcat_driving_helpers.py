from dataclasses import dataclass
import platform

from wildcat_utils import *

@dataclass(frozen=True)
class JoystickConfig:
    X_AXIS : int
    Y_AXIS : int
    RZ_AXIS : int
    LBUMP : int
    RBUMP : int
    X_INV : bool = True
    Y_INV : bool = False
    RZ_INV : bool = True


_JOYSTICK_CFG = {
    "Darwin" : JoystickConfig(X_AXIS=3, Y_AXIS=2, RZ_AXIS=0, LBUMP=6, RBUMP=7),
    "Linux" : JoystickConfig(X_AXIS=4, Y_AXIS=3, RZ_AXIS=0, LBUMP=4, RBUMP=5),
    "Windows" : JoystickConfig(X_AXIS=3, Y_AXIS=2, RZ_AXIS=0, LBUMP=4, RBUMP=5)
    }

JOYSTICK_CFG = _JOYSTICK_CFG[platform.system()]

class SteeringProcessor:
    def __init__(self, cmd_min, cmd_max, cmd_slew_limit):
        self._min = cmd_min
        self._max = cmd_max
        self._slew_limit = cmd_slew_limit

        self._cmd_req      = 0
        self._cmd_d_unfilt = 0
        self._cmd_d        = 0

    @property
    def min(self):
        return self._min

    @property
    def max(self):
        return self._max

    @property
    def slew_limit(self):
        return self._slew_limit

    def saturate(self, val):
        return saturate(val, self._min, self._max)

    @property
    def cmd_req(self):
        return self._cmd_req

    @property
    def cmd_d_unfilt(self):
        return self._cmd_d_unfilt

    @property
    def cmd_d(self):
        return self._cmd_d

    def reset(self, val):
        self._cmd_req = self._cmd_d_unfilt = self._cmd_d = val


class XdSteering(SteeringProcessor):
    def __init__(self, xd_min, xd_max, xd_slew_limit, min_slew_limit, min_slew_vel):
        SteeringProcessor.__init__(self, xd_min, xd_max, xd_slew_limit)

        self._min_slew_limit = min_slew_limit  # The minimum slew limit for xd_d
        self._min_slew_vel   = min_slew_vel    # The velocity at which min slew limit occurs
        self._xd_filter      = None            # A filter for the x velocity

    def update(self, xd_req, dt):
        self._cmd_req    = self.saturate(xd_req)
        derate_factor = max(0.0, -(self._min_slew_limit - self.slew_limit) / self._min_slew_vel) \
            if self._min_slew_vel > 0 else 0.0
        xd_slew_rate = max(self._min_slew_limit, self.slew_limit - abs(self._cmd_d_unfilt) * derate_factor)
        self._cmd_d_unfilt = slew_rate_limit(self.cmd_d_unfilt, self.cmd_req, xd_slew_rate, dt)
        if self._xd_filter:
            self._cmd_d = self._xd_filter.filter_val(self.cmd_d_unfilt)
        else:
            self._cmd_d = self.cmd_d_unfilt
        return self.cmd_d

    def set_filter_params(self, dt, fc, q=None):
        if q:
            self._xd_filter = Filter2ndOrder(dt, fc, 1, q)
        else:
            self._xd_filter = Filter2ndOrder(dt, fc)


class YdSteering(SteeringProcessor):
    def __init__(self, yd_min, yd_max, yd_slew_limit):
        SteeringProcessor.__init__(self, yd_min, yd_max, yd_slew_limit)
        '''pass'''

    def update(self, yd_req, dt):
        self._cmd_req = self.saturate(yd_req)
        self._cmd_d_unfilt = slew_rate_limit(self.cmd_d_unfilt, self.cmd_req, self.slew_limit, dt)
        self._cmd_d = self.cmd_d_unfilt
        return self.cmd_d


class RzdSteering(SteeringProcessor):
    def __init__(self, rzd_min, rzd_max, rzd_slew_limit, rx_max):
        SteeringProcessor.__init__(self, rzd_min, rzd_max, rzd_slew_limit)
        # This class is a bit weird in the sense that it uses the roll limits for some
        # of the turning limits.  We'll store the roll limits here separately.
        # Turning limits are stored in the parent class.
        self._rx_limit = rx_max

        self._rzd_filter = None  # A filter for the slew rate limited rzd_req

    def update(self, rzd_req, xd_d, dt):
        rzd_max = self.max if abs(xd_d) < 0.25 else (9.81 / abs(xd_d)) * math.tan(self._rx_limit)
        self._cmd_req = self.saturate(rzd_req)
        rzd_req       = saturate(self.cmd_req, -rzd_max, rzd_max)

        self._cmd_d_unfilt = slew_rate_limit(self.cmd_d_unfilt, rzd_req, self.slew_limit, dt)
        if self._rzd_filter:
            self._cmd_d = self._rzd_filter.filter_val(self.cmd_d_unfilt)
        else:
            self._cmd_d = self.cmd_d_unfilt

        return self.cmd_d

    def set_filter_params(self, dt, fc, q=None):
        if q:
            self._rzd_filter = Filter2ndOrder(dt, fc, 1, q)
        else:
            self._rzd_filter = Filter2ndOrder(dt, fc)
