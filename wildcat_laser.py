import math

import pygame

from .wildcat_driving_tester import Meter2PixSprite


class Laser(Meter2PixSprite):
    """
    Lasers for the wildcat robot
    """

    def __init__(self, actor, vel, screen, clock):
        """

        @type self: Laser
        @type actor: Meter2PixSprite
        @type vel:
        @type screen: pygame.Surface
        @type clock: pygame.Clock
        """
        Meter2PixSprite.__init__(self)
        # Create an empty surface for this Laser sprite
        self.image = pygame.Surface((1, 1))
        self.rect = pygame.Rect(actor.pospx, (3, 3))
        self.rect.center = actor.pospx

        self._pos = actor.pos
        self._convert_pos()

        self._screen = screen
        self._clock  = clock
        self._yaw = actor.yaw
        self._vel = self.__rot2d((max(vel, 20), 0))
        self._age = 0

        self._oob = False

        actor.reload()

        #print "Laser created at ", self.rect.center

    @property
    def oob(self):
        return self._oob

    @property
    def age(self):
        return self._age

    def update(self):
        dt = self._clock.get_time() / 1000.0
        self._age += dt

        dx = self._vel[0] * dt
        dy = self._vel[1] * dt

        self._pos[0] += dx
        self._pos[1] += dy

        self._convert_pos()
        self.rect.center = self.pospx

        self.draw()

        self.__check_oob()
        if self.oob:
            self.kill()

    def draw(self):
        (px, py) = self.__rot2d([12, 0])
        px += self.rect.centerx
        py += self.rect.centery

        pygame.draw.lines(self._screen, (255, 0, 0), False, [self.rect.center, [px, py]], 2)

    def __rot2d(self, p):
        x_out = math.cos(self._yaw) * p[0] - math.sin(self._yaw) * p[1]
        y_out = math.sin(self._yaw) * p[0] + math.cos(self._yaw) * p[1]
        return x_out, y_out

    def __check_oob(self):
        self._oob = not self._screen.get_rect().collidepoint(self.rect.center)
