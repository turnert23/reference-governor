import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, rc
from scipy.integrate import odeint
import copy
from utilityFunctions import closestCircPoint
class World:
    def __init__(self, center, obstacles = []):
        self.center  = center 
        self.obstacles = obstacles
        self.obspatches = []
        for ob in self.obstacles:
            self.obspatches.append(plt.Circle(ob.center, radius = ob.radius, fc='r', ec = 'k'))
        self.hyperplanes = [] #may be used for detecting voronoi cells  
        # self.find_hyperplanes()
        self.ttmp = 'time = %.1fs'
        self.ttxt = 0.
    def find_hyperplanes(self):
        tmpobstacles = copy.deepcopy(self.obstacles)
        for ob in tmpobstacles:
            tmpobstacles.remove(ob)
            for oth in tmpobstacles:
                alphaij = 1. / 2. - (ob.radius**2 - oth.radius**2)/(2* np.linalg.norm(ob.center - oth.center)**2)
                hij = alphaij * ob.center + (1-alphaij)*oth.center
                h = Hyperplane(hij, np.dot(np.array([[0., -1.],[1., 0.]]), (ob.center - oth.center))*1./np.linalg.norm(ob.center - oth.center))
                self.hyperplanes.append(h)
    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        self.obspatches.append(plt.Circle(obstacle.center, radius = obstacle.radius, fc='r', ec = 'k'))
        # self.find_hyperplanes() # re-compute the set of hyperplanes now that a new obstacle has been added

class CircleWorld(World):
    type = 'circle'
    def __init__(self, center, radius, obstacles=[]):
        super().__init__(center, obstacles)
        self.radius = radius
        self.wpatch = plt.Circle((center), radius = radius, fc= 'none', ec = 'm', ls= '-', linewidth=2.0 )
    def plot(self, fig):
        ax = plt.axes(xlim=(-self.radius-1, self.radius+1), ylim=(-self.radius-1 , self.radius+1))
        ax.add_patch(self.wpatch)
        for obp in self.obspatches:
            ax.add_patch(obp)
        self.ttxt = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        self.ttxt.set_text('')
        return ax
    def update_time(self, t):
        self.ttxt.set_text(self.ttmp % (t))
         
class RectWorld(World):
    type = 'rect'
    def __init__(self, center, dims, obstacles=[]):
        super().__init__(center, obstacles)
        self.length = dims[0]
        self.width = dims[1]
        self.wpatch = plt.Rectangle((self.center[0] - self.length / 2.0, self.center[1] - self.width / 2.0), width = self.length, height = self.width, fc = 'none', ec = 'k', ls = '-' )
    def plot(self, fig):
        ax = plt.axes(xlim = (-self.length / 2.0, self.length /2.0 ), ylim = (-self.width /2.0, self.width/2.0))
        ax.add_patch(self.wpatch)
        for obp in self.obspatches:
            ax.add_patch(obp)
        for h in self.hyperplanes:
            ax.plot((h.point[0] + h.dir[0], 2.0*h.point[0] - 2.0*h.dir[0]),(h.point[1] + 2.0*h.dir[1], h.point[1] - 2.0*h.dir[1]))
        self.ttxt = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        self.ttxt.set_text('')
        return ax
    def update_time(self, t):
        self.ttxt.set_text(self.ttmp % (t))

class Obstacle:
    def __init__(self, center, radius = 1.0):
        self.center = center
        self.radius = radius

class Robot:
    def __init__(self, radius = 0.2):
        self.radius = radius
        self.rpatch = plt.Circle((0., 0.), radius = radius, fc='g')
        self.gpatch = plt.Circle((0., 0.), radius = radius, fc='c')
        self.rapatch = plt.Circle((0., 0.), radius = 0., fc = 'y', alpha = 0.5)
    def draw_at(self, rx, gx):
        self.rpatch.center = (rx[0], rx[1])
        self.gpatch.center = (gx[0], gx[1])
    def draw_init(self,ax):
        ax.add_patch(self.rpatch)
        ax.add_patch(self.gpatch)
    def draw_range(self, gx, world):
        self.rapatch.center = (gx[0], gx[1])
        self.rapatch.radius = closestCircPoint(world.center,world.radius,gx)
    def draw_init_range(self, ax):
        ax.add_patch(self.rapatch)

class Hyperplane:
    def __init__(self, point, direction):
        self.point = point
        self.dir = direction

class Controller:
    def __init__(self, cfunc, goal, params):
        self.function = cfunc
        self.goal = goal
        self.params = params
    def run(self, state, environment):
        newstate = self.function(state, environment, self.goal, self.params)
        return newstate

class Environment:
    def __init__(self,world,robot):
        self.world = world
        self.robot = robot

class Simulation:
    def __init__(self, world, robot, controller, dynamics):
        self.env = Environment (world, robot)
        self.controller = controller
        self.dynamics = dynamics
        self.solution = []
        self.t = []
    def run(self, q0, tStop, tInc):
        self.t = np.arange(0., tStop, tInc)
        params = (self.controller, self.env)
        self.solution = odeint(self.dynamics, q0, self.t, args=(params,))
    def init(self):
        path = np.c_[self.solution[:,0], self.solution[:,1]]
        pathghost = np.c_[self.solution[:,4], self.solution[:,5]]
        self.env.robot.draw_at(path[0,:], pathghost[0,:])
        self.env.robot.draw_init(self.ax)
        self.env.robot.draw_range(pathghost[0,:], self.env.world)
        self.env.robot.draw_init_range(self.ax)
    def animate(self,i):
        path = np.c_[self.solution[:,0], self.solution[:,1]]
        pathghost = np.c_[self.solution[:,4], self.solution[:,5]]
        self.env.robot.draw_at(path[i,:], pathghost[i,:])
        self.env.robot.draw_range(pathghost[i,:], self.env.world)
        self.env.world.update_time(self.t[i])
    def show(self, *args, **kwargs):
        self.vf  = kwargs.get('vf')
        fig = plt.figure()
        plt.axis(scaled=True)
        self.ax = self.env.world.plot(fig)
        ani = animation.FuncAnimation(fig, self.animate, init_func=self.init, frames = len(self.t), interval=20)
        # mng = plt.get_current_fig_manager()
        # mng.window.state('zoomed') 
        plt.show()