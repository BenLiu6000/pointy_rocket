'''
This is a flight simulator for the POINTY rocket
Needs Pygame for visual display

This will be 2 dimensional, however I think it should be possible to go from 2 dimensions to 3 
simply enough when designing for the actual rocket

To do:
optimise PID controller coefficient values


'''





import pygame
import time
from pygame.locals import *
import sys
import math
import random
import os

#remember that in pygame y value goes up as you go lower

pygame.init()

WINDOW = pygame.display.set_mode((1200,600))

'''
For measurements units all measurements will be in:

Metres
Newtons
Seconds
Rotations Per Minute
Kg


'''

#global constants
G = 9.81
INITIAL_DISPLACEMENT = (600,400)
ENGINE_THRUST = 0
ENGINE_THRUST_GRAPH = [
    (0,0),
    (0.049,2.569),
    (0.116,9.369),
    (0.184,17.275),
    (0.237,24.258),
    (0.282,29.73),
    (0.297,27.01),
    (0.311,22.589),
    (0.322,17.99),
    (0.348,14.126),
    (0.386,12.099),
    (0.442,10.808),
    (0.546,9.876),
    (0.718,9.306),
    (0.879,9.105),
    (1.066,8.901),
    (1.257,8.698),
    (1.436,8.31),
    (1.59,8.294),
    (1.612,4.613),
    (1.65,0),
    (1e+67,0)
]
ENGINE_TO_CENTRE_OF_MASS_DISTANCE = 0.3
MAX_ROTATION_SPEED = 0.5
MAX_ROTATION_ANGLE = 2*math.pi

SIMULATION_SCALE = 3 #amount of pixels per metre, eg scale 10 would be 1 metre per 10 pixels

SIMULATION_SPEED = 0.25

SPACING = 25 #number of metres betwen each of the green lines

MAX_HEIGHT = 0

TURBULENCE_AMPLITUDE = 2

#global variables
TIME = 0
 
class Rocket:
    def __init__(self):
        global INITIAL_DISPLACEMENT
        self.position = INITIAL_DISPLACEMENT
        self.velocity = (0,0)
        self.thrustAngle = math.pi/100 #facing directly up is 0 degrees, also in radians, goes clockwise, is the angle between the direction the rocket is pointing and the 
        self.angle = 0
        self.angularVelocity = 0
        self.desiredAngle = 0
        self.centreOfGravity = (0,0)
        self.mass = 1
        self.angleIntegral = 0



def draw_graphics():
    global POINTY, SIMULATION_SCALE, SPACING, TIME

    WINDOW.fill((0,10,30))

    height = 500
    while height>0:
        pygame.draw.line(WINDOW, (0,255,0), (0,height), (1200,height), 2)
        draw_text(f"{round((500-height)/SIMULATION_SCALE)}m",700,height-20)
        height -= SPACING*SIMULATION_SCALE

    playerDisplacement = vec_add(POINTY.position, vec_multiply(INITIAL_DISPLACEMENT,-1))
    playerDisplacementPixels = vec_multiply(playerDisplacement,SIMULATION_SCALE)
    playerPositionInSim = vec_add(playerDisplacementPixels,(300,500))

    #pygame.draw.circle(WINDOW,(150,0,0), playerPositionInSim, 20)
    draw_rocket(playerPositionInSim,POINTY.angle)

    thrustDirection = angle_to_vec(POINTY.thrustAngle+POINTY.angle)
    rocketDirection = angle_to_vec(POINTY.angle)

    forceVector = (thrustDirection[0]*ENGINE_THRUST*3,
                   G + (thrustDirection[1]*ENGINE_THRUST*3) )

    pygame.draw.line(WINDOW, (0,0,200), playerPositionInSim, vec_add(playerPositionInSim, vec_multiply(thrustDirection,100)), 3)
    pygame.draw.line(WINDOW, (200,200,200), playerPositionInSim, vec_add(playerPositionInSim, vec_multiply(rocketDirection,100)), 3)
    #pygame.draw.line(WINDOW, (200,0,200), playerPositionInSim, vec_add(playerPositionInSim, vec_multiply(vec_normalise(forceVector),100)), 3)

    draw_text(f"TIME: {round(TIME,2)}s",900,20)
    draw_text(f"THRUST: {round(ENGINE_THRUST*3,2)}N",900,50)
    draw_text(f"DIST-Y: {round(400-POINTY.position[1],2)}m",900,80)
    draw_text(f"DIST-X: {round(POINTY.position[0]-600,2)}m",900,110)
    draw_text(f"VELOCITY-Y: {round(-POINTY.velocity[1],2)}m/s",900,140)
    draw_text(f"VELOCITY-X: {round(POINTY.velocity[0],2)}m/s",900,170)
    draw_text(f"ANGULAR-VEL: {round(POINTY.angularVelocity)}rad/s",900,200)
    draw_text(f"ANGLE: {round(POINTY.angle,2)}rad",900,230)
    draw_text(f"THRUST-ANG: {round(POINTY.thrustAngle,2)}rad",900,260)
    draw_text(f"ROCKET-MASS: {round(POINTY.mass,2)}kg",900,290)
    draw_text(f"ENGINE-AMT: 3",900,320)
    draw_text(f"CURRENT TURBULENCE: {round(get_turbulence()*POINTY.velocity[1]/POINTY.mass)} idk",800,350)
    draw_text(f"TURBULENCE AMPLITUDE: {TURBULENCE_AMPLITUDE} idk",800,380)





def physics(deltaTime):
    global POINTY, ENGINE_THRUST, TIME

    POINTY.thrustAngle = 0
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        POINTY.thrustAngle = -math.pi/50
    elif keys[pygame.K_RIGHT]:
        POINTY.thrustAngle = math.pi/50
    else:
        POINTY.thrustAngle = PID_controller()

    update_engine_thrust()

    thrustDirection = angle_to_vec(POINTY.thrustAngle+POINTY.angle)
    engineForceVector = ((thrustDirection[0]*ENGINE_THRUST*3,
                          thrustDirection[1]*ENGINE_THRUST*3 ))

    #calculating linear motion
    forceVector = vec_add(engineForceVector,(0,G))
    acceleration = vec_multiply(forceVector, 1/POINTY.mass)

    POINTY.velocity = vec_add(POINTY.velocity, vec_multiply(acceleration, deltaTime/2) )

    POINTY.position = vec_add(POINTY.position, vec_multiply(POINTY.velocity, deltaTime))

    POINTY.velocity = vec_add(POINTY.velocity, vec_multiply(acceleration, deltaTime/2) )

    #rotational motion

    if POINTY.angle > math.pi:
        POINTY.angle -= 2*math.pi
    if POINTY.angle < -math.pi:
        POINTY.angle += 2*math.pi


    torque = get_torque()
    momentOfInertia = POINTY.mass / 12

    angularAccel = torque/momentOfInertia

    POINTY.angularVelocity += angularAccel*deltaTime/2

    POINTY.angle += POINTY.angularVelocity*deltaTime

    POINTY.angularVelocity += angularAccel*deltaTime/2


    POINTY.angularVelocity += get_turbulence()*deltaTime*POINTY.velocity[1]/POINTY.mass

    POINTY.angleIntegral += POINTY.angle*deltaTime


    #floor collision
    if POINTY.position[1] > INITIAL_DISPLACEMENT[1]:
        POINTY.position = (POINTY.position[0], 400)
        POINTY.velocity = (POINTY.velocity[0]*0.3, 0)


def vec_multiply(vec,c):
    return (vec[0]*c, vec[1]*c)

def vec_add(vec1,vec2):
    return (vec1[0]+vec2[0], vec1[1]+vec2[1])

def vec_magnitude(vec):
    return math.sqrt(vec[0]**2 + vec[1]**2)

def vec_normalise(vec): #sets vector magnitude to 1 while keeping same magnitude
    magnitude = vec_magnitude(vec)
    return vec_multiply(vec,1/magnitude)

def vec_dot_product(vec1,vec2):
    return (vec1[0] * vec2[0]) + (vec1[1] * vec2[1])

def angle_to_vec(angle): #0 degrees is facing directly up, goes clockwise around
    return (math.sin(angle),-math.cos(angle)) #y direction is down in pygame

CURRENT_ENGINE_STEP = 0
def update_engine_thrust():
    global ENGINE_THRUST, TIME, CURRENT_ENGINE_STEP, ENGINE_THRUST_GRAPH
    if TIME > ENGINE_THRUST_GRAPH[CURRENT_ENGINE_STEP+1][0]:
        CURRENT_ENGINE_STEP += 1
        ENGINE_THRUST = ENGINE_THRUST_GRAPH[CURRENT_ENGINE_STEP][1]

def get_torque():
    global POINTY
    return math.sin(POINTY.thrustAngle)*ENGINE_THRUST*3*ENGINE_TO_CENTRE_OF_MASS_DISTANCE


def get_proportion():
    global POINTY
    return math.abs(POINTY.thrustAngle-POINTY.desiredAngle)


SYS_FONT = pygame.font.SysFont(pygame.font.get_default_font(),30)
def draw_text(text,x,y):
    textSurface = SYS_FONT.render(text,True,(255,255,255))
    textRect = textSurface.get_rect()
    textRect.left = x
    textRect.top = y
    WINDOW.blit(textSurface,textRect)
    return

ROCKET_IMAGE_EXISTS = False
if os.path.exists("rocketimg.png"):
    ROCKET_IMAGE = pygame.image.load("rocketimg.png")
    ROCKET_IMAGE_EXISTS = True

def draw_rocket(position,angle):
    global ROCKET_IMAGE_EXISTS
    if ROCKET_IMAGE_EXISTS:
        global ROCKET_IMAGE
        scaled = pygame.transform.scale(ROCKET_IMAGE,(80,80))
        rotated = pygame.transform.rotate(scaled, -angle*180/math.pi)

        rect = rotated.get_rect()
        rect.center = (position)
        WINDOW.blit(rotated,rect)
        #WINDOW.blit(rotated,vec_add(position,(-math.sqrt(40)*2*(math.cos(angle)+math.sin(angle)),-math.sqrt(40)*2*(math.cos(angle)-math.sin(angle)))))
    else:
        pygame.draw.circle(WINDOW,(150,0,0), position, 20)
    return



#creates a turbulence map 
TURBULENCE_MAP = [0]
def generate_turbulence(tmap,tamp):
    timems = 0
    while timems < 4000:
        timems += 1
        tmap.append((tmap[timems-1]*0.9) + ((random.random()*tamp) - tamp/2) )
    return tmap

generate_turbulence(TURBULENCE_MAP,TURBULENCE_AMPLITUDE)

def get_turbulence():
    global TURBULENCE_MAP, TIME
    timems = round(TIME*1000)
    if timems > 3500:
        return 0
    return TURBULENCE_MAP[timems]



POINTY = Rocket()

STARTED = False

while not STARTED:
    keys = pygame.key.get_pressed()

    if keys[pygame.K_l]:
        STARTED = True

    draw_graphics()

    draw_text("press l to begin simulation",400,300)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    pygame.display.update()
    pygame.time.Clock().tick(60)


def loop():
    global TIME, SIMULATION_SPEED

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()


        TIME += SIMULATION_SPEED*0.016
        physics(SIMULATION_SPEED*0.016)

        draw_graphics()

        pygame.display.update()
        pygame.time.Clock().tick(60) #just for testing while deltatime is being added

proportionCoefficient = 1
derivativeCoefficient = 0.1
integralCoefficient = 0.01

def PID_controller():
    global POINTY
    motorAngle = 0

    #proportion
    motorAngle += -(POINTY.angle)*proportionCoefficient

    #derivative
    motorAngle += POINTY.angularVelocity*derivativeCoefficient

    #integral
    motorAngle += -(POINTY.angleIntegral)*integralCoefficient

    return motorAngle #return a value for the rocket motor angle (which the servos will turn to)




loop()





