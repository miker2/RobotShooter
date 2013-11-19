import pygame
import math, random, os.path
import numpy
from collections import deque
from bdi.robots.wildcat.wildcat_utils import *
from bdi.robots.wildcat.steering_tester.wildcat_laser import Laser
from bdi.robots.wildcat.steering_tester.wildcat_driving_helpers import *
# import pdb

# Define some colors
black    = (   0,   0,   0)
white    = ( 255, 255, 255)
blue     = (  50,  50, 255)
green    = (   0, 255,   0)
dkgreen  = (   0, 100,   0)
red      = ( 255,   0,   0)
purple   = (0xBF,0x0F,0xB5)
brown    = (0x55,0x33,0x00)

# Define some constants:
PIXELS_PER_METER = 15

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800
GRID_SPACING = int(5.0 * PIXELS_PER_METER);

FPS = 30.0

LASER_VEL = 10.0 * PIXELS_PER_METER
MIN_LASER_AGE = 1/6.0
robot_thk = 3

GRAPH_COLORS = (blue,red,dkgreen,purple)


main_dir = os.path.split(os.path.abspath(__file__))[0]

def load_image(file):
    "loads an image, prepares it for play"
    file = os.path.join(main_dir, file)
    try:
        surface = pygame.image.load(file)
    except pygame.error:
        raise SystemExit('Could not load image "%s" %s'%(file, pygame.get_error()))
    return surface.convert()


# Function to draw the background
def draw_background(screen):
    # Set the screen background
    screen.fill(white)

def draw_grid(screen):
    y_off = SCREEN_HEIGHT % GRID_SPACING
    for y in range(0,SCREEN_HEIGHT+GRID_SPACING,GRID_SPACING):
        pygame.draw.line(screen, dkgreen, [0,y-y_off],[SCREEN_WIDTH,y-y_off],1)
    x_off = SCREEN_WIDTH % GRID_SPACING
    for x in range(0,SCREEN_WIDTH+GRID_SPACING,GRID_SPACING):
        pygame.draw.line(screen, dkgreen, [x-x_off,0],[x-x_off,SCREEN_HEIGHT],1)

def draw_border(screen):
    # Draw a border around the driving area:
    pts = []
    pts.append([0,0])
    pts.append([0,SCREEN_HEIGHT])
    pts.append([SCREEN_WIDTH,SCREEN_HEIGHT])
    pts.append([SCREEN_WIDTH,0])
    pygame.draw.lines(screen, brown, True, pts, 5)

# Kind of a horrible hack
def m2px(val):
    return int(val * PIXELS_PER_METER)
def px2m(val):
    return val / m2px(1.0)


### Define some classes here for the different sprite types.

# The WildCat object isn't a true sprite type because it doesn't use an image to draw itself, 
# but it works for now
class WildCat(pygame.sprite.Sprite):
    RZAXIS = 0
    XAXIS  = 3
    YAXIS  = 4
    DBAND = 0.1
    YDDBAND = 0.1
    XVEL_SCALE =-9.0/(1-DBAND)
    YVEL_SCALE = 0.5/(1-YDDBAND)
    RZD_SCALE  = 1.0/(1-DBAND)
    N_GRAPHS = 2
    def __init__(self,joystick,clock):
        pygame.sprite.Sprite.__init__(self)
        self.image = None
        self._joy = joystick
        self._clock = clock

        self._pos = [px2m(0.5*SCREEN_WIDTH),px2m(0.5*SCREEN_HEIGHT)]
        self._yaw = -math.pi/2

        self.rect = pygame.Rect((m2px(self._pos[0]),m2px(self._pos[1])),(0,0))

        (self._xd_d,self._yd_d,self._rzd_d) = (0,0,0)
        self._screen = pygame.display.get_surface()

        # Set up the steering classes:
        self.xd_steering  = XdSteering(-3.0,9.5,1.5,0.5,7.0)
        self.yd_steering  = YdSteering(-0.5,0.5,0.75)
        self.rzd_steering = RzdSteering(-1.0,1.0,0.4/0.33,0.39)
        # and add some filters:
        self.xd_steering.set_filter_params(1.0/FPS,0.9,0.5)
        self.rzd_steering.set_filter_params(1.0/FPS,3.0,0.5)

        self.xd_steering.reset(0)
        self.yd_steering.reset(0)
        self.rzd_steering.reset(0)

        self._xd_graph  = SteeringGraph(0,'xd',self.xd_steering,self._screen)
        self._rzd_graph = SteeringGraph(1,'rzd',self.rzd_steering,self._screen)

    @property
    def pos(self):
        return self._pos

    @property
    def yaw(self):
        return self._yaw

    def update(self):
        # Process joystick commands here
        self.process_joystick()

        # Update the robot position
        dt = self._clock.get_time() / 1000.0
        self._yaw += self._rzd_d*dt
        dx_b = self._xd_d*dt
        dy_b = self._yd_d*dt
        (dx_w,dy_w) = rot2d(self._yaw,(dx_b,dy_b))

        self._pos[0] += dx_w
        self._pos[1] += dy_w

        # Clamp the robot position to the edges of the screen.
        self._pos[0] = saturate(self._pos[0],0,px2m(SCREEN_WIDTH))
        self._pos[1] = saturate(self._pos[1],0,px2m(SCREEN_HEIGHT))

        # Move the robot
        self.draw()

        # Shoot the laser
        # Not sure if this should be handled here...?

        self._xd_graph.graph()
        self._rzd_graph.graph()

    def process_joystick(self):
        # Get the requested speeds from the joystick
        xd_req  = self.XVEL_SCALE*deadband(self._joy.get_axis(self.XAXIS),-self.DBAND,self.DBAND)
        yd_req  = self.YVEL_SCALE*deadband(self._joy.get_axis(self.YAXIS),-self.YDDBAND,self.YDDBAND)
        rzd_req = self.RZD_SCALE*deadband(self._joy.get_axis(self.RZAXIS),-self.DBAND,self.DBAND)
        # Apply slew rate limits to get the desired speeds
        # do some xd_limit hacks:
        dt = self._clock.get_time() / 1000.0
        self._xd_d  = self.xd_steering.update(xd_req,dt)
        self._yd_d  = self.yd_steering.update(yd_req,dt)
        self._rzd_d = self.rzd_steering.update(rzd_req,self._xd_d,dt)

    def draw(self):
        ''' This is where the drawing of the robot actually happens!'''
        # Setup the robot base drawing:
        (l,w) = (m2px(1.4),m2px(0.6))
        pts = [] # start with empty list
        pts.append([ 0.5*l, 0.5*w])
        pts.append([-0.5*l, 0.5*w])
        pts.append([-0.5*l,-0.5*w])
        pts.append([ 0.5*l,-0.5*w])
        # Convert the passed in position in meters to pixels:
        (xpx,ypx) = [m2px(x) for x in self._pos]
        # Transform the robot from robot coords to world coords:
        for p in pts:
            (xn,yn) = rot2d(self._yaw,p)
            p[0] = xn + xpx; p[1] = yn + ypx;
        pygame.draw.polygon(self._screen, blue, pts, 2)
        (xc,yc) = rot2d(self._yaw,(0.5*l,0))
        pygame.draw.circle(self._screen,green,[int(xc+xpx),int(yc+ypx)],int(0.5*w),0)
        pygame.draw.circle(self._screen,black,[xpx,ypx],2,0)

class LS3(pygame.sprite.Sprite):
    defaultlife = 3
    images = []
    def __init__(self,p0):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.image[0]
        self.rect = self.image.get_rect()
        self.rect.centerx = p0[0]
        self.rect.centery = p0[1]
        self.life = self.defaultlife

    def update(self):
        ''' Update the LS3 position here! '''
        self.life -= 1

        if self.life <= 0: self.kill()


class Explosion(pygame.sprite.Sprite):
    defaultlife = 12
    animcycle = 3
    images = []
    def __init__(self, actor):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = images[0]
        self.rect = self.image.get_rect(center=actor.rect.center)
        self.life = self.defaultlife

    def update(self):
        self.life = self.life - 1
        self.image = self.images[self.life//self.animcycle%2]
        if self.life <= 0: self.kill()


class SteeringGraph:
    GRAPH_HEIGHT = 150
    def __init__(self,gid,name,steering,screen):
        ''' Here is where we'll put all of the graphing data '''
        self._name     = name
        self._gid      = gid
        self._steering = steering
        self._screen   = screen.subsurface((0,SCREEN_HEIGHT+self._gid*self.GRAPH_HEIGHT),
                                           (screen.get_width(),self.GRAPH_HEIGHT))
        self._cmd_d    = deque([],self._screen.get_width())
        self._cmd_req  = deque([],self._screen.get_width())

    def graph(self):
        ''' This is where we'll graph the stuff '''
        self._cmd_d.append(self._steering.cmd_d)
        self._cmd_req.append(self._steering.cmd_req)
        self.draw_joystick_command()
        self.add_labels()

    def add_labels(self):
        # Setup a font for rendering the text
        myFont = pygame.font.Font(pygame.font.match_font("consolas"),16)
        # Label the xd graph
        txt_graph = myFont.render(self._name+"_req",1,GRAPH_COLORS[0],white)
        txt_pos = txt_graph.get_rect()
        txt_pos.x = 10
        txt_pos.top = 10
        self._screen.blit(txt_graph,txt_pos)
        txt_pos_last = txt_pos
        txt_graph = myFont.render(self._name+"_d",1,GRAPH_COLORS[1],white)
        txt_pos = txt_graph.get_rect()
        txt_pos.x = 10
        txt_pos.top = txt_pos_last.bottom
        self._screen.blit(txt_graph,txt_pos)
    
    def draw_joystick_command(self):
        scale = self._screen.get_height() / (self._steering.max-self._steering.min)
        screen_offset = self._screen.get_height()
        min_cmd = self._steering.min
        self._screen.fill(black)
        borderpx = 2
        rect = [borderpx,borderpx,self._screen.get_width()-2*borderpx,self._screen.get_height()-2*borderpx]
        self._screen.fill(white,rect)
        cmd = [self._cmd_req,self._cmd_d]
        if len(cmd[0]) > 2:
            zeros = [[x,screen_offset+scale*min_cmd] for x in range(len(cmd[0]))]
            pygame.draw.lines(self._screen,black,False,zeros,1)
            for c in cmd:
                pts   = [[x,screen_offset-scale*(c[x]-min_cmd)] for x in range(len(c))]
                pygame.draw.lines(self._screen,GRAPH_COLORS[cmd.index(c)],False,pts,2)



def main():
    # Initialize PyGame
    pygame.init()

    # Setup the screen size (including room for graphing
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT+WildCat.N_GRAPHS*SteeringGraph.GRAPH_HEIGHT))

    # Load images, assign to sprite classes
    # (do this before the classes are used, but after the screen is setup).
    LS3.images = [load_image('LS3_clipart.png')]
    img = load_image('explosion1.gif')
    Explosion.images = [img, pygame.transform.flip(img, 1, 1)]

    # Decorate the game window with things like:
    #icon = pygame.transform.scale(load_image('icon_here.png'), (32,32))
    #pygame.display.set_icon(icon)
    #pygame.display.set_caption('WildCat driving simulator')
    #pygame.mouse.set_visible(0)
    
    # Initialize Game Groups
    ls3s      = pygame.sprite.Group()
    lasers    = pygame.sprite.Group()
    allsprite = pygame.sprite.RenderUpdates()
    lastls3   = pygame.sprite.GroupSingle()

    #assign default groups to each sprite class
    LS3.containers    = ls3s, allsprite, lastls3
    Laser.containers  = lasers, allsprite

    # Set up a font for rendering text:
    myFont = pygame.font.Font(pygame.font.match_font("consolas"),16)

    # Count the joysticks the computer has
    joystick_count=pygame.joystick.get_count()
    if joystick_count == 0:
        # No joysticks!
        print ("Error, I didn't find any joysticks.")
        print ("Please connect a joystick to continue.")
        pygame.quit()
    else:
        # pdb.set_trace()
        # Use joystick #0 and initialize it
        my_joystick = pygame.joystick.Joystick(0)
        my_joystick.init()

    # Create a clock
    clock = pygame.time.Clock()
    # Init the clock
    clock.tick()

    wildcat = WildCat(my_joystick,clock)
    # Create a container for the lasers.  A simple list with work.
    lasers = []

    done=False

    while done==False:

        for event in pygame.event.get():
            if event.type == pygame.QUIT or \
                    (event.type == pygame.KEYDOWN and event.key == K_ESCAPE):
                done=True

        # Decorate the game window
        pygame.display.set_caption("FPS: %.2f" % (clock.get_fps()))

        # As long as there is a joystick
        if joystick_count != 0:

            if (my_joystick.get_button(4) or my_joystick.get_button(5)) and (not lasers or lasers[-1].age >= MIN_LASER_AGE):
                #laser_vel = LASER_VEL+xd_d*PIXELS_PER_METER
                laser_vel = LASER_VEL
                lasers.append( Laser((m2px(wildcat.pos[0]),m2px(wildcat.pos[1])), wildcat.yaw, 
                                     laser_vel, screen.subsurface(0,0,SCREEN_WIDTH,SCREEN_HEIGHT)) )

            dt = clock.get_time() / 1000.0

        draw_background(screen)
        draw_grid(screen)

        # Draw the item at the proper coordinates
        wildcat.update()
    
        for l in lasers:
            l.update(dt)
            if l.oob:
                lasers.remove(l)

        draw_border(screen)


        # Here we'll display some metrics to the driver:
        txt_xd  = myFont.render("xd_req  = % .2f | xd_d  = % .2f" % (wildcat.xd_steering.cmd_req,
                                                                     wildcat.xd_steering.cmd_d), 
                                1, black, white)
        txt_xd_pos = txt_xd.get_rect()
        txt_xd_pos.x = 10
        txt_xd_pos.top = 5
        screen.blit(txt_xd,txt_xd_pos)
        txt_yd  = myFont.render("yd_req  = % .2f | yd_d  = % .2f" % (wildcat.yd_steering.cmd_req,
                                                                     wildcat.yd_steering.cmd_d), 
                                1, black, white)
        txt_yd_pos = txt_yd.get_rect()
        txt_yd_pos.x = 10
        txt_yd_pos.top = txt_xd_pos.bottom
        screen.blit(txt_yd,txt_yd_pos)
        txt_rzd = myFont.render("rzd_req = % .2f | rzd_d = % .2f" % (wildcat.rzd_steering.cmd_req,
                                                                     wildcat.rzd_steering.cmd_d), 
                                1, black, white)
        txt_rzd_pos = txt_rzd.get_rect()
        txt_rzd_pos.x = 10
        txt_rzd_pos.top = txt_yd_pos.bottom
        screen.blit(txt_rzd,txt_rzd_pos)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

# call the "main" function if running this script
if __name__ == '__main__': main()
