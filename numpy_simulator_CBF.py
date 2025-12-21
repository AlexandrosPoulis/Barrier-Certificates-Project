import pygame
import sympy as sp
import numpy as np
from numpy import sum
# For QP solver
from scipy.optimize import minimize


class vehicle():
    def __init__(self,x,y,height,width,color):
        self.x = float(x)
        self.y = float(y)
        self.height = height
        self.width = width
        self.color = color
        # Simpler Unicycle dynamics
        # Value of theta doesn't seem to matter on the initialazation state
        self.theta =  0.0
        self.v = 0.0
        self.maxv = 7.5# Worked fine at both 3 and 7.5, change to maxv for a unicycle in meters/second
        
        # The maxv also affects how fast the vehicle detects the barrier for some reason
        # so a big velocity for the car would mean detecting the barrier too early
        self.omega = 0.0
        self.maxomega = 25# Worked fine for 15 and 25 
        # kept to print the trajectory taken so far
        self.trajectory = [[self.x,self.y]]

        self.rect = pygame.Rect(x,y,height,width)

    def draw(self,simulator):
        # Draw vehicle
        self.rect.center = (self.x,self.y)
        simulator.window.fill(self.color,self.rect)
        for i in range (len(self.trajectory)-1):
            pygame.draw.line(simulator.window, (0,0,205), (self.trajectory[i][0],self.trajectory[i][1]),
                             (self.trajectory[i+1][0],self.trajectory[i+1][1]),2)
    
    def move(self,state):
        self.trajectory.append([self.x,self.y])
        self.x = state[0]
        self.y = state[1]
        self.theta = state[2]

class obstacles():
    def __init__(self, num, pos, obst_h, color):
        self.num = num
        self.obst = []
        self.color = color
        for i in range(self.num):
            self.obst.append(obstacle(pos[i], obst_h, color))
    
    def draw(self,simulator):
        for i in range(self.num):
            # Uncomment to show radius * 2, for some reason that the radius considered by the BF
            #pygame.draw.circle(surface=simulator.window, color=(242, 210, 189), 
            #                   center=(self.obst[i].x,self.obst[i].y), radius=2*simulator.rad) 
            self.obst[i].rect.center  = (self.obst[i].x,self.obst[i].y)
            simulator.window.fill(self.obst[i].color,self.obst[i].rect)  

class obstacle():
    def __init__(self,pos,obst_h,color):
        self.x = pos[0]
        self.y = pos[1]
        self.h = obst_h
        self.color = color
        self.rect = pygame.Rect(self.x,self.y,self.h,self.h)

class simulator():
    def __init__(self,window=[800,800],color=(0,0,0),veh_pos=[300,700],veh_color=(0,0,255)
                 ,no_obst=1,obst_pos=[[280,300]],obst_h=10,obst_color=(255,0,0),radius=30,goal_color=(0,255,0), goal = [300,0]):
        # Numbers for the different tests done(other than the obstacles positions everything stayd the same):
        # case 1: [[100,100]],
        # case 2: [[280,300]],
        # case 3: [[280,300],[700,400]],
        # case 4: [[280,300], [400,300]]
        # Change the obstacle_ to any of the above and also change no_obst to represent the number
        # of obstacles to test any of the above cases
        
        self.x1_sym, self.x2_sym = sp.symbols("x1 x2")
        self.x = window[0]
        self.y = window[1]
        self.color = color
        self.goalcolor = goal_color
        self.rad = radius
        self.veh = vehicle(veh_pos[0],veh_pos[1],4,4,veh_color)
        self.obst = obstacles(no_obst,pos=obst_pos,obst_h=obst_h,color=obst_color)
        
        # change to change goal
        self.goalx = goal[0]
        self.goaly = goal[1]

        # K-function parameter used in CBF
        # Closest to 0 avoids as har as possible
        # close to 1 avoids "last minute"
        self.alpha = 0.9

        # Simulation parameter
        self.dt = 0.1

    def run_sim(self):
        pygame.init()
        self.window = pygame.display.set_mode((self.x ,self.y))
        pygame.display.set_caption("Simulator")
        
        runninggame = True
        while runninggame:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    runninggame = False
            self.window.fill(self.color)
            u = self.QP()
            self.veh.v, self.veh.omega,delta = u

            state_dot = np.array([
                self.veh.v * np.cos(self.veh.theta),
                (self.veh.v * np.sin(self.veh.theta)),
                self.veh.omega # θ = ω
            ])  
            
            state = [self.veh.x,self.veh.y,self.veh.omega] + state_dot * self.dt
            # Append to a list if you wanna have the trajectory drawn out as well
            self.veh.move(state)
            self.ReDrawWindow()
            pygame.display.update()

    
    def ReDrawWindow(self):
        # Draw goal
        temp_goal = pygame.Rect(self.goalx,self.goaly,10,10)
        self.window.fill(self.goalcolor,temp_goal)

        # Draw obstacles
        self.obst.draw(self)

        # Draw vehicle
        self.veh.draw(self)

    def grad_B(self,car_pos,obst_pos):
        # Gradient of the BF
        return 2 * (car_pos - obst_pos)
    
    def B_i(self, p, po):
        return sum((p - po)**2) - (2*self.rad)**2

    def f(self):
        # ===========================================
        # Unicycle models have no an f(x) of 0
        # apparently f is the drift dynamics
        # ===========================================
        return 0.0
        
    def g(self,grad):
        # ===========================================
        # g in this case represents the dynamics used
        # by the unicycle
        # ===========================================
        g_pos = np.array([[np.cos(self.veh.theta), 0],
                          [np.sin(self.veh.theta), 0]])
        return grad @ g_pos

    def nominal_control(self):
        car_pos = np.array([self.veh.x, self.veh.y])
        goal_pos = np.array([self.goalx, self.goaly])
        
        to_goal = goal_pos - car_pos
        dist_to_goal = np.linalg.norm(to_goal)

        # If close to obstacle, add tangential avoidance
        desired_direction = to_goal / dist_to_goal

        desired_theta = np.arctan2(desired_direction[1], desired_direction[0])
        theta_error = np.arctan2(np.sin(desired_theta - self.veh.theta), 
                                    np.cos(desired_theta - self.veh.theta))
        # Nominal control
        v_nom = dist_to_goal
        omega_nom = desired_theta
        omega_nom = np.clip(omega_nom, -self.veh.maxomega, self.veh.maxomega)

        return np.array([v_nom, omega_nom])
    
    def QP(self):
        u_nom = self.nominal_control()
        # TO CHANGE
        h = []
        grad = []
        Lf_h = []
        Lg_h = []
        for i in range (self.obst.num):
            car_pos = np.array([self.veh.x, self.veh.y])
            obst_pos = np.array([self.obst.obst[i].x, self.obst.obst[i].y])
            goal_pos = np.array([self.goalx, self.goaly])


            h.append(self.barrier_total(car_pos, obst_pos))
            grad.append(self.grad_B(car_pos,obst_pos))
            
            
            Lf_h.append(self.f())
            Lg_h.append(self.g(grad[i]))
        bounds = [(0, self.veh.maxv),  # v must be non-negative
                (-self.veh.maxomega, self.veh.maxomega),
                (0, None)] # there shoudln't be a max for delta
            
        def cbf_constraint(z,i):#Lf_h,Lg_h, h):
            # CBF condition: Lf·h + Lg·h·u + α·h + δ ≥ 0
            u = z[:2]
            delta = z[2]
            return Lf_h[i] + Lg_h[i] @ u + self.alpha * h[i] + delta
        

        def objective(z):
            # The function that QP is solving for
            rho = 1e5
            u = z[:2]
            delta = z[2]
            return np.sum((u - u_nom)**2 + (rho*delta)**2)


        # Constraints
        constraints = []
        for i in range(self.obst.num):
            constraints.append({
                'type': 'ineq',
                'fun': lambda u, i=i: cbf_constraint(u,i)# Lf_h[i],Lg_h[i], h[i])
            })

        delta = 0.0
        result = minimize(objective, [u_nom[0], u_nom[1], delta], method='SLSQP', constraints=constraints,
                         bounds=bounds,
                         options={'maxiter': 400, 'ftol': 1e-11})
        #print(result.success)
        if result.success:
            #print("Succ")
            #print(result.x[2])
            return result.x
        else:
            # Large ρ or very tight CBF bounds can trigger this.

            # No value of u was found that satisfies the constaraints so we return [0,0,0]
            # mostly since if we didn't the simulator would crash, but also [0,0,0] means that
            # essentialy there will be no movement, and if the QP failed with the previous attempt to find u
            # it would fail again essentialy still presenting the same deadlock that would realistically happen
            # in this case
            return [0.0, 0.0, 0.0]#result.x#[u_nom[0], u_nom[1], 0.0]#u_nom
        
    # ----- Total barrier -----
    def barrier_total(self,veh_pos_jax,obst_pos_jax):
        barrier = self.B_i(veh_pos_jax,obst_pos_jax)
        #print(barrier)
        return self.B_i(veh_pos_jax,obst_pos_jax)

sim = simulator()

sim.run_sim()

