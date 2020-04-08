import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, rc
from scipy.integrate import odeint
import string
from simObjects import CircleWorld, Obstacle, Robot
from utilityFunctions import closestCircPoint

class GuideDot:
    def __init__(self, radius = 0.1):
        self.radius = radius
        self.dotpatch = plt.Circle((0., 0.), radius = radius, fc='g')
    def draw_at(self, rx):
        self.dotpatch.center = (rx[0], rx[1])
    def draw_init(self,ax):
        ax.add_patch(self.dotpatch)

class PenTip:
    def __init__(self, radius = 0.1):
        self.radius = radius
        self.ptpatch = plt.Circle((0., 0.), radius = radius, fc='g')
        self.gpatch = plt.Circle((0., 0.), radius = radius, fc='c')
        self.grpatch = plt.Circle((0., 0.), radius = 0., fc = 'y', alpha = 0.5)
    def draw_at(self, rx, gx):
        self.ptpatch.center = (rx[0], rx[1])
        self.gpatch.center = (gx[0], gx[1])
    def draw_init(self,ax):
        ax.add_patch(self.ptpatch)
        ax.add_patch(self.gpatch)
    def draw_range(self, gx, world):
        self.grpatch.center = (gx[0], gx[1])
        self.grpatch.radius = closestCircPoint(world.center,world.radius,gx)
    def draw_init_range(self, ax):
        ax.add_patch(self.rapatch)

def controller(x, xg, current_stroke, pen_tip, cparams):
    #probably want to substitute local free space for curvature. 
    # kappa, kg, eta = cparams
    guide_to_tip_potential = (x - xg) / np.linalg.norm(x-xg) if np.linalg.norm(x-xg) != 0 else 0  
    # minWsGoal = xg + 1.0 * lineToHip
    fx = -2. * kappa*(x-xg) # - eta * xdot 
    r = current_stroke.find_gradient(xg)
    # r = -((2. * nbeta * (xg-xstar) - 1./k * np.linalg.norm(xg - xstar)**2 * ngradbeta )/ ((np.linalg.norm(xg-xstar)**(2. * k) + nbeta)**(1./k +1)))
    EofX = (kappa * np.linalg.norm(x-xg)**2)
    # deltaE = kappa * (closestCircPoint(world.center, world.radius, xg) - robot.radius )**2 - EofX #np.linalg.norm(closestCircPoint(world.center,world.radius,xg)-xg)**2 - EofX
    # r = 0.5 * (xstar - xg)
    minTerm = min(np.linalg.norm(r), np.sqrt(EofX)/kappa)
    gx = kg * r/ np.linalg.norm(r) * minTerm if np.linalg.norm(r) != 0 else np.array([0.,0.])
    return (fx,gx) 

def dynamics(q,t,params):
    x = q[0:2]
    # xdot = q[2:4]
    xg = q[2:]
    # xstar, world, cparams = params
    fx, gx = controller(x,xg,current_stroke, pen_tip, cparams)
    return [fx[0], fx[1], gx[0], gx[1]]

def dot_dynamics(q,t):
    theta = q[0]
    r = 1 + 0.5 * np.sin(5*theta) #manifold coordinates, may have to find these from world frame in future 
    k = np.abs(r**2 + 2. * 25 * 0.25 * np.cos(5*theta)**2 - r * - 25*0.25*np.sin(5*theta)) / (r**2 + 25*0.25*np.cos(5*theta)**2)**(3./2.) 
    cmd = 5 - 0.25 * k if np.sign(2. * np.pi - theta) >= 0 else 0.
    return [cmd] #constant velocity in theta
    
q0 = [0.0]
tStop = 2000
tInc = 0.05
t = np.arange(0., tStop, tInc)
gdot = GuideDot(0.1)
psoln = odeint(dot_dynamics, q0, t)
theta = psoln[:,0]
r = 1. + 0.5* np.sin(theta)
path = np.c_[r*np.cos(theta), r*np.sin(theta)]
fig = plt.figure()
ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
# ax.add_patch(self.wpatch)
# for obp in self.obspatches:
#     ax.add_patch(obp)
# self.ttxt = ax.text(0.05, 0.9, '', transform=ax.transAxes)
# self.ttxt.set_text('')

def init():
    # plt.title('With Arm Penetration Potential ON')
    gdot.draw_at(path[0,:])
    gdot.draw_init(ax)

def animate(i):
    gdot.draw_at(path[i,:])
    plt.plot(path[0:i,0],path[0:i,1],'k')

ani = animation.FuncAnimation(fig, animate, init_func=init, frames = len(t), interval=20)
plt.show()
