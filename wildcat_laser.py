import pygame
import math

class Laser:
    """
    Lasers for the wildcat robot
    """

    def __init__(self,pos,yaw,vel,screen):
        self.screen = screen
        self.pos = list(pos)
        self.yaw = yaw
        self.vel = self.__rot2d((max(vel,20),0))
        self._age = 0

        self._oob = False

    @property
    def oob(self):
        return self._oob

    @property
    def age(self):
        return self._age

    def update(self,dt):
        self._age += dt

        dx = self.vel[0] * dt;
        dy = self.vel[1] * dt;

        self.pos[0] += dx
        self.pos[1] += dy

        (px,py) = self.__rot2d([12,0])
        px += self.pos[0]
        py += self.pos[1]

        pygame.draw.lines(self.screen, (255,0,0),False,[self.pos,[px,py]],2)

        self.__check_oob()

    def __rot2d(self,p):
        x_out = math.cos(self.yaw) * p[0] - math.sin(self.yaw) * p[1]
        y_out = math.sin(self.yaw) * p[0] + math.cos(self.yaw) * p[1]
        return (x_out,y_out)

    def __check_oob(self):
        self._oob &= self.pos[0] < 0
        self._oob &= self.pos[0] > self.screen.get_width()
        self._oob &= self.pos[1] < 0
        self._oob &= self.pos[1] > self.screen.get_width()
