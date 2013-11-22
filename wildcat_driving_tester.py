import pygame
import math
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
METERS_PER_PIXEL = 1.0 / PIXELS_PER_METER

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800
GRID_SPACING = int(5.0 * PIXELS_PER_METER);

FPS = 30.0

DBAND = 0.1
YDDBAND = 0.1
XVEL_SCALE =-9.0/(1-DBAND)
YVEL_SCALE = 0.5/(1-YDDBAND)
RZD_SCALE  = 1.0/(1-DBAND)
GRAPH_HEIGHT = 150 # height of graph in pixels.

LASER_VEL = 10.0 * PIXELS_PER_METER
MIN_LASER_AGE = 1/6.0
robot_thk = 3

GRAPH_COLORS = (blue,red,dkgreen,purple)

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

def convert_pos_to_pix(x,y):
    # Convert robot position to pixels
    xpx = x*PIXELS_PER_METER
    ypx = y*PIXELS_PER_METER
    # Saturate robot position so it doesn't run off the screen
    xpx = saturate(xpx,0,screen.get_width())
    ypx = saturate(ypx,0,screen.get_height())
    return (int(xpx),int(ypx))

def draw_robot(screen,x,y,yaw,thk):
    # Setup the robot base drawing:
    (l,w) = (1.4*PIXELS_PER_METER,0.6*PIXELS_PER_METER)
    pts = [] # start with empty list
    pts.append([ 0.5*l, 0.5*w])
    pts.append([-0.5*l, 0.5*w])
    pts.append([-0.5*l,-0.5*w])
    pts.append([ 0.5*l,-0.5*w])
    # Convert the passed in position in meters to pixels:
    (xpx,ypx) = convert_pos_to_pix(x,y)
    # Transform the robot from robot coords to world coords:
    for p in pts:
        (xn,yn) = rot2d(yaw,p)
        p[0] = xn + xpx; p[1] = yn + ypx;
    pygame.draw.polygon(screen, blue, pts, thk)
    (xc,yc) = rot2d(yaw,(0.5*l,0))
    pygame.draw.circle(screen,green,[int(xc+xpx),int(yc+ypx)],int(0.5*w),0)
    pygame.draw.circle(screen,black,[xpx,ypx],2,0)

def draw_joystick_command(screen,cmd,steering,graph_id):
    scale = GRAPH_HEIGHT / (steering.max-steering.min)
    screen_offset = screen.get_height() - graph_id * GRAPH_HEIGHT
    min_cmd = steering.min
    rect = [0,screen_offset-GRAPH_HEIGHT,screen.get_width(),GRAPH_HEIGHT]
    #pygame.draw.rect(screen,white,rect)
    screen.fill(white,rect)
    if len(cmd[0]) > 2:
        zeros = [[x,screen_offset+scale*min_cmd] for x in range(len(cmd[0]))]
        pygame.draw.lines(screen,black,False,zeros,1)
        for c in cmd:
            pts   = [[x,screen_offset-scale*(c[x]-min_cmd)] for x in range(len(c))]
            pygame.draw.lines(screen,GRAPH_COLORS[cmd.index(c)],False,pts,2)


# Set up the steering classes:
xd_steering  = XdSteering(-3.0,9.5,1.5,0.5,7.0)
yd_steering  = YdSteering(-0.5,0.5,0.75)
rzd_steering = RzdSteering(-1.0,1.0,0.4/0.33,0.39)
# and add some filters:
xd_steering.set_filter_params(1.0/FPS,0.9,0.5)
rzd_steering.set_filter_params(1.0/FPS,3.0,0.5)

xd_steering.reset(0)
yd_steering.reset(0)
rzd_steering.reset(0)

# Create a container for the lasers.  A simple list with work.
lasers = []

# Set up some containers for graphing some values:
data_xd_req = deque([], SCREEN_WIDTH)
data_xd_d   = deque([], SCREEN_WIDTH)
N_GRAPHS = 1 # because all of this will be on the same graph
data_rzd_req = deque([], SCREEN_WIDTH)
data_rzd_d   = deque([], SCREEN_WIDTH)
N_GRAPHS += 1

# Initialize PyGame
pygame.init()

# Setup the screen size (including room for graphing
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT+N_GRAPHS*GRAPH_HEIGHT))

# Set up a font for rendering text:
myFont = pygame.font.Font(pygame.font.match_font("consolas"),16)

# Count the joysticks the computer has
joystick_count=pygame.joystick.get_count()
if joystick_count == 0:
    # No joysticks!
    print ("Error, I didn't find any joysticks.")
else:
    # pdb.set_trace()
    # Use joystick #0 and initialize it
    my_joystick = pygame.joystick.Joystick(0)
    my_joystick.init()

# Create a clock
clock = pygame.time.Clock()
# Init the clock
clock.tick()
done=False

# Set initial conditions:
x_pos = METERS_PER_PIXEL * SCREEN_WIDTH * 0.5
y_pos = METERS_PER_PIXEL * SCREEN_HEIGHT * 0.5
yaw     = -math.pi/2

while done==False:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done=True

    pygame.display.set_caption("FPS: %.2f" % (clock.get_fps()))

    # As long as there is a joystick
    if joystick_count != 0:

        # This gets the position of the axis on the game controller
        # It returns a number between -1.0 and +1.0
        joy_lft_x = my_joystick.get_axis(0)
        joy_lft_y = my_joystick.get_axis(1)
        joy_shldr = my_joystick.get_axis(2)
        joy_rht_x = my_joystick.get_axis(3)
        joy_rht_y = my_joystick.get_axis(4)

        (hx,hy) = my_joystick.get_hat(0)
        #robot_thk += hy
        #print "Hat %d position is (%d,%d)" % (0,hx,hy)
                
        #for hat in range(1,my_joystick.get_numhats()):
        #    (hx,hy) = my_joystick.get_hat(hat)
        #    print "Hat %d position is (%d,%d)" % (hat,hx,hy)

        for b in range(0,my_joystick.get_numbuttons()):
            if my_joystick.get_button(b):
                print "Detected button press on button %d" % b

        if (my_joystick.get_button(4) or my_joystick.get_button(5)) and (not lasers or lasers[-1].age >= MIN_LASER_AGE):
            #laser_vel = LASER_VEL+xd_d*PIXELS_PER_METER
            laser_vel = LASER_VEL
            lasers.append( Laser((x_pos*PIXELS_PER_METER,y_pos*PIXELS_PER_METER), yaw, 
                                 laser_vel, screen) )

        dt = clock.get_time() / 1000.0

        # Get the requested speeds from the joystick
        xd_req  = XVEL_SCALE*deadband(joy_rht_x,-DBAND,DBAND)
        yd_req  = YVEL_SCALE*deadband(joy_rht_y,-YDDBAND,YDDBAND)
        rzd_req = RZD_SCALE*deadband(joy_lft_x,-DBAND,DBAND)
        # Apply slew rate limits to get the desired speeds
        # do some xd_limit hacks:
        xd_d  = xd_steering.update(xd_req,dt)
        yd_d  = yd_steering.update(yd_req,dt)
        rzd_d = rzd_steering.update(rzd_req,xd_d,dt)

        data_xd_req.append(xd_steering.cmd_req)
        data_xd_d.append(xd_d)
        data_rzd_req.append(rzd_steering.cmd_req)
        data_rzd_d.append(rzd_d)

        yaw = yaw+rzd_d*dt
        dx_b = xd_d*dt
        dy_b = yd_d*dt
        (dx_w,dy_w) = rot2d(yaw,(dx_b,dy_b))

        x_pos=x_pos+dx_w
        y_pos=y_pos+dy_w
        print "joy_rht_x = %f, joy_rht_y = %f, joy_lft_x = %f" % (joy_rht_x,joy_rht_y,joy_lft_x)
        print "x_pos  = % f, y_pos  = % f, yaw     = % f" % (x_pos,y_pos,yaw)
        print "dx_b   = % f, dy_b   = % f"                % (dx_b, dy_b)
        print "dx_w   = % f, dy_w   = % f"                % (dx_w, dy_w)
        print "xpx    = % d, ypx    = % d"                % (x_pos*PIXELS_PER_METER,y_pos*PIXELS_PER_METER)

    draw_background(screen)
    draw_grid(screen)

    # Draw the item at the proper coordinates
    draw_robot(screen,x_pos,y_pos,yaw,robot_thk)

    for l in lasers:
        l.update(dt)
        if l.oob:
            lasers.remove(l)

    draw_border(screen)

    draw_joystick_command(screen,(data_xd_req,data_xd_d),xd_steering,0)
    draw_joystick_command(screen,(data_rzd_req,data_rzd_d),rzd_steering,1)

    # Label the xd graph
    txt_graph = myFont.render("xd_req",1,GRAPH_COLORS[0],white)
    txt_pos = txt_graph.get_rect()
    txt_pos.x = 10
    txt_pos.centery = screen.get_height() - 1*GRAPH_HEIGHT + txt_pos.height + 0
    screen.blit(txt_graph,txt_pos)
    txt_pos_last = txt_pos
    txt_graph = myFont.render("xd_d",1,GRAPH_COLORS[1],white)
    txt_pos = txt_graph.get_rect()
    txt_pos.x = 10
    txt_pos.top = txt_pos_last.bottom
    screen.blit(txt_graph,txt_pos)
    
    # Label the rzd graph
    txt_graph = myFont.render("rzd_cmd",1,GRAPH_COLORS[0],white)
    txt_pos = txt_graph.get_rect()
    txt_pos.x = 10
    txt_pos.centery = screen.get_height() - 2*GRAPH_HEIGHT + txt_pos.height + 0
    screen.blit(txt_graph,txt_pos)
    txt_pos_last = txt_pos
    txt_graph = myFont.render("rzd_d",1,GRAPH_COLORS[1],white)
    txt_pos = txt_graph.get_rect()
    txt_pos.x = 10
    txt_pos.top = txt_pos_last.bottom
    screen.blit(txt_graph,txt_pos)


    # Here we'll display some metrics to the driver:
    txt_xd  = myFont.render("xd_req  = % .2f | xd_d  = % .2f" % (xd_req,xd_d), 1, black, white)
    txt_xd_pos = txt_xd.get_rect()
    txt_xd_pos.x = 10
    txt_xd_pos.top = 5
    screen.blit(txt_xd,txt_xd_pos)
    txt_yd  = myFont.render("yd_req  = % .2f | yd_d  = % .2f" % (yd_req,yd_d), 1, black, white)
    txt_yd_pos = txt_yd.get_rect()
    txt_yd_pos.x = 10
    txt_yd_pos.top = txt_xd_pos.bottom
    screen.blit(txt_yd,txt_yd_pos)
    txt_rzd = myFont.render("rzd_req = % .2f | rzd_d = % .2f" % (rzd_req,rzd_d), 1, black, white)
    txt_rzd_pos = txt_rzd.get_rect()
    txt_rzd_pos.x = 10
    txt_rzd_pos.top = txt_yd_pos.bottom
    screen.blit(txt_rzd,txt_rzd_pos)

    # Just in case we hit a wall, saturate the position
    x_pos = saturate(x_pos,0,SCREEN_WIDTH*METERS_PER_PIXEL)
    y_pos = saturate(y_pos,0,SCREEN_HEIGHT*METERS_PER_PIXEL)

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
