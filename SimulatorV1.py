import pygame
from BFCreationConnected import BFCreation
import time
from math import sin, cos, tan, radians
from jax import grad
import jax.numpy as jnp
from numpy import abs
import sympy as sp


class car:
    def __init__(self,x,y,height,width,color):
        self.x = float(x)
        self.y = float(y)
        self.height = height
        self.width = width
        self.color = color
        # Dynamics
        self.lr = 1
        self.v = 10 # velocity
        self.b = radians(6)
        self.ps = self.v/self.lr * tan(self.b)

        self.rect = pygame.Rect(x,y,height,width)

    def draw(self):
        self.rect.topleft = (self.x,self.y)
        window.fill(self.color,self.rect)

    def move(self,x,y):
        if x == 1:
            self.x = self.x + (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        if x == -1:
            self.x = self.x - (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        if y == 1:
           self.y = self.y - (self.v * (sin(self.ps) - cos(self.ps) * tan(self.b)))
    
    def move1(self,dir):
        self.x += dir[1] * (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        self.y += dir[0] * (self.v * (sin(self.ps) - cos(self.ps) * tan(self.b)))

class obstacle:
    def __init__ (self,x,y,height,width,color):
        self.x = float(x)
        self.y = float(y)
        self.height = height
        self.width = width
        self.color = color
        self.rect = pygame.Rect(x,y,height,width)
    def draw(self):
        self.rect.topleft = (self.x,self.y)
        window.fill(self.color,self.rect)

#class obstacles:
#    def __init__ (self, ):

def distance(car,obst,barrier):

    # need to make it work for a list of obst not just for 1
    grad = direction_dx(car,obst[0],barrier)
    cliped_grad = jnp.clip(grad, -1.0, 1.0)
    print(grad)
    car.move1(cliped_grad)
    #print(temp)

    

def direction_dx(car,obst,barrier):
    car_pos = jnp.array([car.x, car.y])
    obst_pos = jnp.array([obst.x, obst.y])


    # ----- Gradient of total barrier -----
    grad_total = grad(barrier_total, argnums=0)
    gradient = grad_total(car_pos,obst_pos, 5, barrier)
    #print(gradient)
    return gradient

# ----- Quadratic barrier -----
def barrier_quad(x, barrier):
    x1, x2 = x[0], x[1]
    return barrier(x1,x2)

# ----- Circular obstacle barrier -----
def barrier_circle(car, obstacle, rad):
    eps = 1e-8
    d = jnp.linalg.norm(car - obstacle) - rad
    d = jnp.maximum(d, eps)  # avoid log(0) or negative
    return -jnp.log(d)


def make_barrier_fn(expr):
    return sp.lambdify((x1_sym, x2_sym), expr, "jax")


# ----- Total barrier -----
def barrier_total(car, obstacle, rad, barrier):
    x1_sym, x2_sym = sp.symbols("x1 x2")
    barrier_fn = make_barrier_fn(barrier)
    return barrier_quad(car, barrier_fn) + barrier_circle(car, obstacle, rad)

    
x1_sym, x2_sym = sp.symbols("x1 x2")

pygame.init()

wx = 800
wy = 800
window = pygame.display.set_mode((wx,wy))
pygame.display.set_caption("Simulator")


white = (255,255,255)
red = (254,0,0)
ob_h = 10
car1 = car(300,500,10,10,white)
obst1 = obstacle(300,50,ob_h,ob_h,red)
#obst2 = obstacle(400,50,ob_h,ob_h,red)
#obst3 = obstacle(400,175,ob_h,ob_h,red)

lst_obst = [obst1]#, obst2, obst3]
obst = [[obst1.x, obst1.y]]#, [obst2.x, obst2.y], [obst3.x, obst3.y]]

max_d, bf, gamma, lamb = BFCreation([wx,wy], obst, [car1.x, car1.y], ob_h)
print(type(bf))

def ReDrawWindow(car1,obst1,bf):
    distance(car1,lst_obst,bf)
    for obs in lst_obst:
        obs.draw()
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

