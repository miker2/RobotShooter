import pygame
import math, random, os.path, copy
import numpy
from collections import deque
from wildcat_utils import *
#from wildcat_laser import Laser
from wildcat_driving_helpers import *
# import pdb

# Define some colors
black = (   0, 0, 0)
white = ( 255, 255, 255)
blue = (  50, 50, 255)
green = (   0, 255, 0)
dkgreen = (   0, 100, 0)
red = ( 255, 0, 0)
purple = (0xBF, 0x0F, 0xB5)
brown = (0x55, 0x33, 0x00)

# Define some constants:
PIXELS_PER_METER = 15

SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 720
GRID_SPACING = int(5.0 * PIXELS_PER_METER);

FPS = 30.0

MIN_LASER_AGE = 1 / 6.0
MAX_SHOTS = 10

LS3_ODDS   = 22 # Chances a new LS3 appears
LS3_RELOAD = int(2 * FPS) # Frames between new LS3s

GRAPH_COLORS = (blue, red, dkgreen, purple)

main_dir = os.path.split(os.path.abspath(__file__))[0]


def load_image(filename, colorkey=None):
    "loads an image, prepares it for play"
    filename = os.path.join(main_dir, filename)
    try:
        surface = pygame.image.load(filename)
    except pygame.error, message:
        raise SystemExit('Could not load image "%s" %s' % (filename, pygame.get_error()))
    image = surface.convert()
    if colorkey is not None:
        if colorkey is -1:
            colorkey = image.get_at((0, 0))
        image.set_colorkey(colorkey, pygame.RLEACCEL)
    return image


def load_images(*files):
    imgs = []
    for file in files:
        imgs.append(load_image(file, -1))
    return imgs


def draw_background(screen):
    # Function to draw the background
    # Set the screen background
    screen.fill(white)


def draw_grid(screen):
    y_off = SCREEN_HEIGHT % GRID_SPACING
    for y in range(0, SCREEN_HEIGHT + GRID_SPACING, GRID_SPACING):
        pygame.draw.line(screen, dkgreen, [0, y - y_off], [SCREEN_WIDTH, y - y_off], 1)
    x_off = SCREEN_WIDTH % GRID_SPACING
    for x in range(0, SCREEN_WIDTH + GRID_SPACING, GRID_SPACING):
        pygame.draw.line(screen, dkgreen, [x - x_off, 0], [x - x_off, SCREEN_HEIGHT], 1)


def draw_border(screen):
    # Draw a border around the driving area:
    pts = []
    pts.append([0, 0])
    pts.append([0, SCREEN_HEIGHT])
    pts.append([SCREEN_WIDTH, SCREEN_HEIGHT])
    pts.append([SCREEN_WIDTH, 0])
    pygame.draw.lines(screen, brown, True, pts, 5)


def m2px(val):
    # Convert meters to pixels
    return int(val * PIXELS_PER_METER)


def px2m(val):
    # Convert pixels to meters
    return val / m2px(1.0)


### Define some classes here for the different sprite types.
class Meter2PixSprite(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self, self.containers)

        # Initialize some base members
        self._pospx = (0, 0)
        self._pos = [0, 0]

    @property
    def pospx(self):
        return self._pospx

    def pos(self):
        return self._pos

    def _convert_pos(self):
        self._pospx = tuple([m2px(x) for x in self._pos])


# The WildCat object isn't a true sprite type because it doesn't use an image to draw itself, 
# but it works for now
class WildCat(Meter2PixSprite):
    RZAXIS = 0
    XAXIS = 3
    YAXIS = 4
    DBAND = 0.1
    YDDBAND = 0.1
    XVEL_SCALE = -9.0 / (1 - DBAND)
    YVEL_SCALE = 0.5 / (1 - YDDBAND)
    RZD_SCALE = 1.0 / (1 - DBAND)
    N_GRAPHS = 2
    RELOAD_TIME = 1 / 6.0
    DIMS = (m2px(1.4), m2px(0.6))

    def __init__(self, joystick, clock):
        Meter2PixSprite.__init__(self)
        self._joy = joystick
        self._clock = clock

        self._pos = [px2m(0.5 * SCREEN_WIDTH), px2m(0.5 * SCREEN_HEIGHT)]
        self._yaw = -math.pi / 2

        self._convert_pos()
        rsize = 2 * max(self.DIMS[0],self.DIMS[1])
        self.rect = pygame.Rect(self.pospx, (rsize, rsize))
        # Hack together a surface to overwrite our last position.
        self.image = pygame.Surface(self.rect.size)
        self.image.fill(white)
        self.image.set_colorkey(white)

        (self._xd_d, self._yd_d, self._rzd_d) = (0, 0, 0)
        self._screen = pygame.display.get_surface()

        self._reload = self.RELOAD_TIME

        # Set up the steering classes:
        self.xd_steering = XdSteering(-3.0, 9.5, 1.5, 0.5, 7.0)
        self.yd_steering = YdSteering(-0.5, 0.5, 0.75)
        self.rzd_steering = RzdSteering(-1.0, 1.0, 0.4 / 0.33, 0.39)
        # and add some filters:
        self.xd_steering.set_filter_params(1.0 / FPS, 0.9, 0.5)
        self.rzd_steering.set_filter_params(1.0 / FPS, 3.0, 0.5)

        self.xd_steering.reset(0)
        self.yd_steering.reset(0)
        self.rzd_steering.reset(0)

        self._xd_graph = SteeringGraph(0, 'xd', self.xd_steering, self._screen)
        self._rzd_graph = SteeringGraph(1, 'rzd', self.rzd_steering, self._screen)

    @property
    def pos(self):
        return self._pos

    @property
    def pospx(self):
        return tuple([m2px(e) for e in self._pos])

    @property
    def yaw(self):
        return self._yaw

    @property
    def reloading(self):
        return self._reload >= 0

    def reload(self):
        self._reload = self.RELOAD_TIME

    def update(self):
        # Process joystick commands here
        self.process_joystick()

        # Update the robot position
        dt = self._clock.get_time() / 1000.0
        self._yaw += self._rzd_d * dt
        dx_b = self._xd_d * dt
        dy_b = self._yd_d * dt
        (dx_w, dy_w) = rot2d(self._yaw, (dx_b, dy_b))

        self._pos[0] += dx_w
        self._pos[1] += dy_w

        # Clamp the robot position to the edges of the screen.
        self._pos[0] = saturate(self._pos[0], 0, px2m(SCREEN_WIDTH))
        self._pos[1] = saturate(self._pos[1], 0, px2m(SCREEN_HEIGHT))

        self._reload -= dt

        self._convert_pos()
        self.rect.center = self.pospx
        # Move the robot
        self.draw()

        # Shoot the laser
        # Not sure if this should be handled here...?

        self._xd_graph.graph()
        self._rzd_graph.graph()

    def process_joystick(self):
        # Get the requested speeds from the joystick
        xd_req = self.XVEL_SCALE * deadband(self._joy.get_axis(self.XAXIS), -self.DBAND, self.DBAND)
        yd_req = self.YVEL_SCALE * deadband(self._joy.get_axis(self.YAXIS), -self.YDDBAND, self.YDDBAND)
        rzd_req = self.RZD_SCALE * deadband(self._joy.get_axis(self.RZAXIS), -self.DBAND, self.DBAND)
        # Apply slew rate limits to get the desired speeds
        # do some xd_limit hacks:
        dt = self._clock.get_time() / 1000.0
        self._xd_d = self.xd_steering.update(xd_req, dt)
        self._yd_d = self.yd_steering.update(yd_req, dt)
        self._rzd_d = self.rzd_steering.update(rzd_req, self._xd_d, dt)

    def draw(self):
        ''' This is where the drawing of the robot actually happens!'''
        (xpx, ypx) = self.pospx
        # Setup the robot base drawing:
        (l, w) = self.DIMS
        pts = [] # start with empty list
        pts.append([0.5 * l, 0.5 * w])
        pts.append([-0.5 * l, 0.5 * w])
        pts.append([-0.5 * l, -0.5 * w])
        pts.append([0.5 * l, -0.5 * w])
        pts.append([ 0.5*l+0.5*w, 0])
        # Transform the robot from robot coords to world coords:
        for p in pts:
            (xn, yn) = rot2d(self._yaw, p)
            p[0] = xn + xpx;
            p[1] = yn + ypx;
        pygame.draw.polygon(self._screen, blue, pts, 2)
        (xc, yc) = rot2d(self._yaw, (0.5 * l, 0))
        #pygame.draw.circle(self._screen,green,[int(xc+xpx),int(yc+ypx)],int(0.5*w),0)
        pygame.draw.circle(self._screen, black, [xpx, ypx], 2, 0)
        self.create_rect(pts)


    def create_rect(self,pts):
        buf  = 2
        xmin = xmax = self.rect.centerx
        ymin = ymax = self.rect.centery
        for p in pts:
            xmax = max(xmax,p[0])
            xmin = min(xmin,p[0])
            ymax = max(ymax,p[1])
            ymin = min(ymin,p[1])
        rect = pygame.Rect((xmin-buf/2,ymin-buf/2),(xmax-xmin+buf,ymax-ymin+buf))
        self.rect = rect
        newpts = [rect.bottomleft,rect.topleft,rect.topright,rect.bottomright]
        pygame.draw.polygon(self._screen,red,newpts,1)

    


class Laser(Meter2PixSprite):
    """
    Lasers for the wildcat robot
    """
    LASER_VEL = (10, 0) # meters / sec
    LASER_LEN = 14 # px

    def __init__(self, actor, screen, clock):
        Meter2PixSprite.__init__(self)
        # Create an empty surface for this Laser sprite
        self.rect = pygame.Rect(actor.pospx, (self.LASER_LEN, self.LASER_LEN))
        self.rect.center = actor.pospx
        self.image = pygame.Surface(self.rect.size)
        self.image.set_alpha(0)

        self._pos = copy.deepcopy(actor.pos)
        self._convert_pos()

        self._screen = screen
        self._clock = clock
        self._vec = rot2d(actor.yaw, (self.LASER_LEN, 0))
        self._vel = rot2d(actor.yaw, self.LASER_VEL)
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

        dx = self._vel[0] * dt;
        dy = self._vel[1] * dt;

        self._pos[0] += dx
        self._pos[1] += dy

        self._convert_pos()
        self.rect.center = self.pospx

        self.draw()

        self.__check_oob()
        if self.oob:
            self.kill()

    def draw(self):
        (px, py) = self._vec
        px += self.rect.centerx
        py += self.rect.centery

        pygame.draw.lines(self._screen, (255, 0, 0), False, [self.rect.center, [px, py]], 2)

    def __check_oob(self):
        self._oob = not self._screen.get_rect().collidepoint(self.rect.center)

    def check_collision(laser, actor):
        return actor.rect.collidepoint(laser.pospx)


class LS3(Meter2PixSprite):
    defaultlife = 3
    ticksperimg = int(0.5 * FPS)
    SCREENRECT = pygame.Rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT)
    SCREENRECTMETER = pygame.Rect(0, 0, px2m(SCREEN_WIDTH), px2m(SCREEN_HEIGHT))
    images = []

    def __init__(self, p0):
        Meter2PixSprite.__init__(self)
        self.image = self.images[0]
        self.rect = self.image.get_rect()
        self._pos = [px2m(x) for x in p0]
        self._convert_pos()
        self.rect.center = self.pospx
        self.life = self.defaultlife
        # Keep track of which image we're on.
        self.frame = 0

        self._randwalk = copy.deepcopy(self._pos)
        self._xfilt = Filter2ndOrder(1.0/FPS,0.05)
        self._yfilt = Filter2ndOrder(1.0/FPS,0.05)

    def update(self):
        ''' Update the LS3 position here! '''
        self.frame += 1
        self._randwalk[0] += random.choice([-1,1]) * 10.4 / FPS
        self._randwalk[1] += random.choice([-1,1]) * 10.4 / FPS
        x_old = self._pos[0]
        self._pos[0] = self._xfilt.filter_val(self._randwalk[0])
        self._pos[1] = self._yfilt.filter_val(self._randwalk[1])
        self._convert_pos()
        self.rect.center = self.pospx
        if not self.SCREENRECT.contains(self.rect):
            self.rect = self.rect.clamp(self.SCREENRECT)
            self._pos = [px2m(self.rect.centerx), px2m(self.rect.centery)]
        dx = self._pos[0] - x_old
        if dx <= 0:
            _dir = 0
        elif dx > 0:
            _dir = 2

        # TODO: Add a bias into the LS3 velocity that causes them to walk toward
        # the wildcat robot
        # TODO: Fix jitter in image. Maybe do this by not changing the _dir variable
        # unless the value of self.frame//self.ticksperimg%2 changes

        self.image = self.images[self.frame // self.ticksperimg % 2 + _dir]

        if self.life <= 0: self.kill()


class Explosion(pygame.sprite.Sprite):
    defaultlife = 12
    animcycle = 3
    images = []

    def __init__(self, actor):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect(center=actor.rect.center)
        self.life = self.defaultlife

    def update(self):
        self.life = self.life - 1
        self.image = self.images[self.life // self.animcycle % 2]
        if self.life <= 0: self.kill()


class SteeringGraph:
    GRAPH_HEIGHT = 150

    def __init__(self, gid, name, steering, screen):
        ''' Here is where we'll put all of the graphing data '''
        self._name = name
        self._gid = gid
        self._steering = steering
        self._screen = screen.subsurface((0, SCREEN_HEIGHT + self._gid * self.GRAPH_HEIGHT),
                                         (screen.get_width(), self.GRAPH_HEIGHT))
        self._cmd_d = deque([], self._screen.get_width())
        self._cmd_req = deque([], self._screen.get_width())

    def graph(self):
        ''' This is where we'll graph the stuff '''
        self._cmd_d.append(self._steering.cmd_d)
        self._cmd_req.append(self._steering.cmd_req)
        self.draw_joystick_command()
        self.add_labels()

    def add_labels(self):
        # Setup a font for rendering the text
        myFont = pygame.font.Font(pygame.font.match_font("consolas"), 16)
        # Label the xd graph
        txt_graph = myFont.render(self._name + "_req", 1, GRAPH_COLORS[0], white)
        txt_pos = txt_graph.get_rect()
        txt_pos.x = 10
        txt_pos.top = 10
        self._screen.blit(txt_graph, txt_pos)
        txt_pos_last = txt_pos
        txt_graph = myFont.render(self._name + "_d", 1, GRAPH_COLORS[1], white)
        txt_pos = txt_graph.get_rect()
        txt_pos.x = 10
        txt_pos.top = txt_pos_last.bottom
        self._screen.blit(txt_graph, txt_pos)

    def draw_joystick_command(self):
        scale = self._screen.get_height() / (self._steering.max - self._steering.min)
        screen_offset = self._screen.get_height()
        min_cmd = self._steering.min
        self._screen.fill(black)
        borderpx = 2
        rect = [borderpx, borderpx, self._screen.get_width() - 2 * borderpx, self._screen.get_height() - 2 * borderpx]
        self._screen.fill(white, rect)
        cmd = [self._cmd_req, self._cmd_d]
        if len(cmd[0]) > 2:
            zeros = [[x, screen_offset + scale * min_cmd] for x in range(len(cmd[0]))]
            pygame.draw.lines(self._screen, black, False, zeros, 1)
            for c in cmd:
                pts = [[x, screen_offset - scale * (c[x] - min_cmd)] for x in range(len(c))]
                pygame.draw.lines(self._screen, GRAPH_COLORS[cmd.index(c)], False, pts, 2)


def main():
    # Initialize PyGame
    pygame.init()

    # Setup the screen size (including room for graphing
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT + WildCat.N_GRAPHS * SteeringGraph.GRAPH_HEIGHT))

    # Load images, assign to sprite classes
    # (do this before the classes are used, but after the screen is setup).
    img = load_images('LS3_FLHR_small.png', 'LS3_FRHL_small.png')
    LS3.images = img + [pygame.transform.flip(im, 1, 0) for im in img]
    img = load_image('explosion1.gif', -1)
    Explosion.images = [img, pygame.transform.flip(img, 1, 1)]

    # Decorate the game window with things like:
    #icon = pygame.transform.scale(load_image('icon_here.png'), (32,32))
    #pygame.display.set_icon(icon)
    #pygame.display.set_caption('WildCat driving simulator')
    #pygame.mouse.set_visible(0)

    background = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
    draw_background(background)
    draw_grid(background)
    draw_border(background)
    screen.blit(background, (0, 0))
    pygame.display.flip()

    # Initialize Game Groups
    player = pygame.sprite.GroupSingle()
    ls3s = pygame.sprite.Group()
    lasers = pygame.sprite.Group()
    allsprite = pygame.sprite.RenderUpdates()
    lastls3 = pygame.sprite.GroupSingle()

    #assign default groups to each sprite class
    WildCat.containers = allsprite
    LS3.containers = ls3s, allsprite, lastls3
    Laser.containers = lasers, allsprite
    Explosion.containers = allsprite

    # Set up a font for rendering text:
    myFont = pygame.font.Font(pygame.font.match_font("consolas"), 16)

    #Setup some game variables
    EasterEggMode = False;
    Ls3Reload = LS3_RELOAD

    # Count the joysticks the computer has
    joystick_count = pygame.joystick.get_count()
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

    wildcat = WildCat(my_joystick, clock)
    ### Create a container for the lasers.  A simple list with work.
    ##lasers = []

    done = False

    print wildcat.alive()

    pygame.key.set_repeat() # Disables key repeats.

    while not done: # wildcat.alive():

        for event in pygame.event.get():
            if event.type == pygame.QUIT or \
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                done = True
            if event.type == pygame.KEYDOWN and event.key == pygame.K_e:
                EasterEggMode = not EasterEggMode

        keystate = pygame.key.get_pressed()

        # Decorate the game window
        caption = "FPS: %.2f" % (clock.get_fps())
        if EasterEggMode:
            caption = caption + "  -  Get the LS3s!"
            
            if Ls3Reload:
                Ls3Reload -= 1
            elif not int(random.random() * LS3_ODDS):
                LS3((random.randint(0,SCREEN_WIDTH),random.randint(0,SCREEN_HEIGHT)))
                Ls3Reload = LS3_RELOAD


        pygame.display.set_caption(caption)


        
        #if keystate[pygame.K_l] and EasterEggMode:
        #        LS3((random.randint(0,SCREEN_WIDTH),random.randint(0,SCREEN_HEIGHT)))


        # As long as there is a joystick
        if joystick_count != 0:
            # if (my_joystick.get_button(4) or my_joystick.get_button(5)):
            #     print "Do no lasers exist?  ", not lasers
            #     print "There are %d lasers." % len(lasers)
            #     print wildcat.reloading
            if (my_joystick.get_button(4) or my_joystick.get_button(5) or keystate[pygame.K_SPACE]) and (
                not wildcat.reloading) and (len(lasers) < MAX_SHOTS):
                Laser(wildcat, screen.subsurface(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), clock)

        dt = clock.get_time() / 1000.0

        allsprite.clear(screen, background)

        allsprite.update()

        if not EasterEggMode and len(ls3s) > 0:
            for rbt in ls3s:
                Explosion(rbt)
                rbt.kill()
            ls3s.empty()

        # Check for laser to robot collisions
        #for rbt in pygame.sprite.groupcollide(lasers, ls3s, 1, 1).keys():
        #    Explosion(rbt)
        # A bit hacky, but use collidepoint to do collision check.o
        for l in lasers:
            for rbt in pygame.sprite.spritecollide(l, ls3s, 1, Laser.check_collision):
                Explosion(rbt)
                rbt.kill()
                l.kill()

        # Check for wildcat to robot collisions
        for rbt in pygame.sprite.spritecollide(wildcat, ls3s, 1):
            Explosion(wildcat)
            Explosion(rbt)
            wildcat.kill()
            rbt.kill()

        #for l in lasers:
        #    l.draw()

        dirty = allsprite.draw(screen)
        pygame.display.update(dirty)


        # Here we'll display some metrics to the driver:
        txt_xd = myFont.render("xd_req  = % .2f | xd_d  = % .2f" % (wildcat.xd_steering.cmd_req,
                                                                    wildcat.xd_steering.cmd_d),
                               1, black, white)
        txt_xd_pos = txt_xd.get_rect()
        txt_xd_pos.x = 10
        txt_xd_pos.top = 5
        screen.blit(txt_xd, txt_xd_pos)
        txt_yd = myFont.render("yd_req  = % .2f | yd_d  = % .2f" % (wildcat.yd_steering.cmd_req,
                                                                    wildcat.yd_steering.cmd_d),
                               1, black, white)
        txt_yd_pos = txt_yd.get_rect()
        txt_yd_pos.x = 10
        txt_yd_pos.top = txt_xd_pos.bottom
        screen.blit(txt_yd, txt_yd_pos)
        txt_rzd = myFont.render("rzd_req = % .2f | rzd_d = % .2f" % (wildcat.rzd_steering.cmd_req,
                                                                     wildcat.rzd_steering.cmd_d),
                                1, black, white)
        txt_rzd_pos = txt_rzd.get_rect()
        txt_rzd_pos.x = 10
        txt_rzd_pos.top = txt_yd_pos.bottom
        screen.blit(txt_rzd, txt_rzd_pos)

        pygame.display.flip()
        clock.tick(FPS)

    print lasers

    pygame.quit()

# call the "main" function if running this script
if __name__ == '__main__': main()
