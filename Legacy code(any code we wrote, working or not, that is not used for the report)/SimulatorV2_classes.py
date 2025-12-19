import pygame
import time
from math import sin, cos, tan, radians
from BFCreationConnected import BFCreation
import sympy as sp
from math import sin, cos, tan, radians, dist
from jax import grad
from numpy import abs
import jax.numpy as jnp
class vehicle():
    def __init__(self,x,y,height,width,color):
        self.x = float(x)
        self.y = float(y)
        self.height = height
        self.width = width
        self.color = color
        # Dynamics
        self.lr = 0.1
        self.v = 4 # velocity
        self.b = radians(6)
        self.ps = self.v/self.lr * tan(self.b)

        self.rect = pygame.Rect(x,y,height,width)

    def draw(self,simulator):
        #pygame.Rect.move
        self.rect.center = (self.x,self.y)
        simulator.window.fill(self.color,self.rect)
    
    def move(self,dir):
        #dt = pygame.clock.tick(60)/1000#dir[1] *#dir[0] *
        self.x += dir[1] * (self.v * (cos(self.ps) - sin(self.ps) * tan(self.b)))
        self.y += dir[0] * (self.v * (sin(self.ps) - cos(self.ps) * tan(self.b)))

class obstacles():
    def __init__(self, num, pos, obst_h, color):
        self.num = num
        self.obst = []
        self.color = color
        for i in range(self.num):
            self.obst.append(obstacle(pos[i], obst_h[i], color))
    
    def draw(self,simulator):
        for i in range(self.num):
            self.obst[i].rect.center  = (self.obst[i].x,self.obst[i].y)
            simulator.window.fill(self.obst[i].color,self.obst[i].rect)   

class obstacle():
    def __init__(self,pos,obst_h,color):
        #print(pos[1])
        self.x = pos[0]
        self.y = pos[1]
        self.h = obst_h
        self.color = color
        self.rect = pygame.Rect(self.x,self.y,self.h,self.h)

class simulator():
    def __init__(self,window=[800,800],color=(255,255,255),veh_pos=[300,500],veh_color=(255,0,0)
                 ,no_obst=1,obst_pos=[[300,150]],obst_h=[10],obst_color=(255,0,0),radius=80):
        
        self.x1_sym, self.x2_sym = sp.symbols("x1 x2")
        self.x = window[0]
        self.y = window[1]
        self.color = color
        self.rad = radius
        self.veh = vehicle(veh_pos[0],veh_pos[1],10,20,veh_color)
        self.obst = obstacles(no_obst,obst_pos,obst_h,obst_color)

    def run_sim(self):
        pygame.init()
        self.window = pygame.display.set_mode((self.x ,self.y))
        #self.window.fill(self.color)
        pygame.display.set_caption("Simulator")
        self.max_d, self.bf, self.gamma, self.lamb = BFCreation([self.x,self.y], self.obst, 
                                                                [self.veh.x, self.veh.y])
        runninggame = True
        while runninggame:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    runninggame = False
            time.sleep(0.05)
            #self.window.fill(self.color)

            self.ReDrawWindow()
            pygame.display.update()

    def ReDrawWindow(self):
        self.distance()
        self.veh.draw(self)
        for obs in self.obst.obst:
            self.obst.draw(self)
    
    def distance(self):
        temp_ob_dist = []
        self.max = 0
        c = 0
        for ob in self.obst.obst:
            temp_dist = dist([self.veh.x,self.veh.y],[ob.x,ob.y])
            temp_ob_dist.append(temp_dist)
            if temp_ob_dist[self.max] >= temp_dist:
                self.max = c
            c += 1
        
        # need to make it work for a list of obst not just for 1
        gradient = self.direction_dx()
        cliped_grad = jnp.clip(gradient, -1.0, 1.0)
        #clip because some times the vehicle accelerated for no reason
        self.veh.move(self.lamb * cliped_grad)#self.gradient/cliped_grad # self.gamma *
        print(cliped_grad)
        

    def direction_dx(self):
        car_pos = jnp.array([self.veh.x, self.veh.y])
        obst_pos = jnp.array([self.obst.obst[self.max].x, self.obst.obst[self.max].y])


        # ----- Gradient of total barrier -----
        grad_total = grad(self.barrier_total, argnums=0)
        return grad_total(car_pos,obst_pos)

    # ----- Quadratic barrier -----
    def barrier_quad(self, veh_pos_jax,barrier):
        return barrier(veh_pos_jax[0], veh_pos_jax[1])

    # ----- Circular obstacle barrier -----
    def barrier_circle(self, veh_pos_jax,obst_pos_jax):
        eps = 1e-8
        d = jnp.linalg.norm(veh_pos_jax - obst_pos_jax) - self.rad
        d = jnp.maximum(d, eps)  # avoid log(0) or negative
        #return -jnp.log(d)
        return self.lamb * (1.0 / (d ** 2))#could change the power from 1 to 2 
    
    def make_barrier_fn(self):
        return sp.lambdify((self.x1_sym, self.x2_sym), self.bf, "jax")

    # ----- Total barrier -----
    def barrier_total(self,veh_pos_jax,obst_pos_jax):
        barrier = self.make_barrier_fn()
        return self.lamb * self.barrier_quad(veh_pos_jax,barrier) + self.barrier_circle(veh_pos_jax,obst_pos_jax)
        #self.lamb * 

#sim = simulator()
#sim.run_sim()