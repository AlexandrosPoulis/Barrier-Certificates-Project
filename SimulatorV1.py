import pygame
from BFCreationConnected import BFCreation
import time
from math import sin, cos, tan, radians
from jax import grad
import jax.numpy as jnp
pygame.init()

wx = 800
wy = 800
window = pygame.display.set_mode((wx,wy))
pygame.display.set_caption("Simulator")

class car:
    def __init__(self,x,y,height,width,color):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.color = color
        # Dynamics
        self.lr = 1
        self.v = 10 # velocity
        self.b = radians(6)
        self.ps = self.v/self.lr * tan(self.b)
        #self.carimage = pygame.image.load("1.png")
        self.rect = pygame.Rect(x,y,height,width)
        #self.rect = pygame.Rect(x,y,height,width)
    def draw(self):
        self.rect.topleft = (self.x,self.y)
        window.fill(self.color,self.rect)
        #self.rect.topleft = (self.x,self.y)
        #window.blit(self.carimage,self.rect)

    def move(self,x,y):
        if x == 1:
            self.x = self.x + (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        if x == -1:
            self.x = self.x - (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        if y == 1:
           self.y = self.y - (self.v * (sin(self.ps) - cos(self.ps) * tan(self.b)))
 

class obstacle:
    def __init__ (self,x,y,height,width,color):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.color = color
        self.rect = pygame.Rect(x,y,height,width)
    def draw(self):
        self.rect.topleft = (self.x,self.y)
        window.fill(self.color,self.rect)

def distance(car,obst,barrier):
    x = car.x - obst.x
    #print( abs(x))
    y = car.y - obst.y
    #print(y)
    if abs(x) + abs(y) <= 100:
        if x < 10:
            if x > 0:
                car.move(1,0)
            if x <= 0:
                car.move(-1,0)
    else:
        car1.move(0,1)
    #print(distance_dx(car,obst,barrier))

def distance_BF(x, x_o, r_o):
    return jnp.linalg.norm(x - x_o) - r_o

def barrier1(x, x_o, r_o):
    d = distance_BF(x, x_o, r_o)
    return -jnp.log(d)

def distance_dx(car,obst,barrier):
    temp = dx(car,obst,barrier)
    print(temp)

def dx(car,obstacle,barrier):
    rad = 1000
    dBdx = grad(barrier)
    #dBdx(car, obstacle, rad)
    return dBdx(car, obstacle, rad)

white = (255,255,255)
red = (254,0,0)
car1 = car(300,500,10,10,white)
obst1 = obstacle(300,50,30,30,red)

bf = BFCreation([wx,wy], [obst1.x, obst1.y])
print(bf)
def ReDrawWindow(car1,obst1,bf):
    distance(car1,obst1,bf)
    obst1.draw()
    car1.draw()

# main loop
runninggame = True
while runninggame:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            runninggame = False
    time.sleep(0.05)
    #window.fill(white)
    ReDrawWindow(car1,obst1,bf)
    pygame.display.update()

pygame.quit()
