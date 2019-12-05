import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, rc
from scipy.integrate import odeint
import string
from simObjects import CircleWorld, Obstacle, Robot
from utilityFunctions import closestCircPoint

def vfPlanner(xg,xstar, world, robot):
    
    numer  =   10. * np.linalg.norm(xg-xstar)**2
    gradnumer = 2. * 10. * (xg - xstar)
    denom = np.linalg.norm(xg-xstar)**2 + (world.radius - robot.radius)**2 - np.linalg.norm(xg-world.center)**2
    graddenom = 2. * (world.center - xstar)
    r = -(denom * gradnumer  - graddenom * numer) / (denom**2)
    return r


def controller(x,xdot, xg, xstar,  world, robot, cparams):
    kappa, kg, eta = cparams
    obs = world.obstacles
    fx = -2. * kappa*(x-xg) - eta * xdot 
    r = vfPlanner(xg, xstar, world, robot)
    # r = -((2. * nbeta * (xg-xstar) - 1./k * np.linalg.norm(xg - xstar)**2 * ngradbeta )/ ((np.linalg.norm(xg-xstar)**(2. * k) + nbeta)**(1./k +1)))
    EofX = (0.5 * np.linalg.norm(xdot)**2 + kappa * np.linalg.norm(x-xg)**2)
    deltaE = kappa * (closestCircPoint(world.center, world.radius, xg) - robot.radius )**2 - EofX #np.linalg.norm(closestCircPoint(world.center,world.radius,xg)-xg)**2 - EofX
    # r = 0.5 * (xstar - xg)
    minTerm = min(np.linalg.norm(r), np.sqrt(deltaE)/kappa)
    gx = kg *  r/ np.linalg.norm(r) * minTerm
    return (fx,gx) 

def dynamics(q,t,params):
    x = q[0:2]
    xdot = q[2:4]
    xg = q[4:]
    xstar, world, cparams = params
    fx, gx = controller(x,xdot,xg,xstar, world, robot, cparams)
    return [xdot[0], xdot[1], fx[0], fx[1], gx[0], gx[1]]

q0 = [4.,3.5,0.,0.,4.,3.5]
tStop = 2000
tInc = 0.05
t = np.arange(0., tStop, tInc)
obsts = [] # ((np.array([2.5,1.25]),0.5),(np.array([3.0,3.0]),0.5))
world = CircleWorld(np.array([0.,0.]), 6., obsts)
robot = Robot(0.2)
goal = np.array([-5.0, 0.])
kappa = 1.
kg = 2.
eta = 1.
gains = (kappa, kg, eta)
params = (goal, world, gains)
psoln = odeint(dynamics, q0, t, args=(params,))

path = np.c_[psoln[:,0], psoln[:,1]]
pathghost = np.c_[psoln[:,4], psoln[:,5]]

def vfPlotter(world, goal, vfplan):
    Xf = np.linspace(world.center[0] - world.radius, world.center[0] + world.radius,24)
    Yf = np.linspace(world.center[1] - world.radius, world.center[1] + world.radius,24)
    X = np.array([])
    Y = np.array([])
    U = np.array([])
    V = np.array([])
    for x in Xf:
        for y in Yf:
            if(np.linalg.norm([x,y] - world.center) < world.radius - 0.2):
                u,v = vfplan(np.array([x,y]), goal, world, robot)
                X = np.append(X,x)
                Y = np.append(Y,y)
                U = np.append(U,u)
                V = np.append(V,v)
    return X,Y,U,V

def getMagGrid(U,V):
    Z = np.zeros([len(U),len(V)]) 
    for u in range(len(U)):
        for v in range(len(V)):
            Z[u,v] = np.log10(np.sqrt(U[u]**2 + V[v]**2))
    return Z

fig = plt.figure()
fig.set_size_inches(7,6.5)
ax = world.plot(fig)

def init():
    # plt.title('With Arm Penetration Potential ON')
    X,Y,U,V = vfPlotter(world, goal, vfPlanner)
    mag = np.sqrt(U**2 + V**2)
    ax.quiver(X,Y,U/mag,V/mag,np.log10(mag), cmap="viridis", width = 0.003)
    robot.draw_at(path[0,:], pathghost[0,:])
    robot.draw_init(ax)
    robot.draw_range(pathghost[0,:], world)
    robot.draw_init_range(ax)

def animate(i):
    robot.draw_at(path[i,:], pathghost[i,:])
    robot.draw_range(pathghost[i,:], world)
    world.update_time(t[i])


ani = animation.FuncAnimation(fig, animate, init_func=init, frames = len(t), interval=20)
plt.show()

