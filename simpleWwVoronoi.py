import numpy as np
import string
import bisect
from simObjects import *

def govController(state, environment, goal, cparams):   
    x = state[0:2]
    xdot = state[2:4]
    xg = state[4:]
    rbt = environment.robot
    wrld = environment.world
    kappa, kg, eta = cparams
    fx = -2. * kappa*(x-xg) - eta * xdot 
    numer  =   10. * np.linalg.norm(xg-goal)**2
    gradnumer = 2. * 10. * (xg - goal)
    denom = np.linalg.norm(xg-goal)**2 + (wrld.radius - rbt.radius)**2 - np.linalg.norm(xg-wrld.center)**2
    graddenom = 2. * (wrld.center - goal)
    r = -(denom * gradnumer  - graddenom * numer) / (denom**2) 
    EofX = (0.5 * np.linalg.norm(xdot)**2 + kappa * np.linalg.norm(x-xg)**2)
    deltaE = kappa * (closestCircPoint(wrld.center,wrld.radius,xg) - 0.2 )**2 - EofX 
    minTerm = min(np.linalg.norm(r), np.sqrt(deltaE)/kappa)
    gx = kg *  r/ np.linalg.norm(r) * minTerm
    return (fx,gx) 

def dynamics(q,t, params):
    ctrlr, env = params
    xstar = ctrlr.goal
    cfunc = ctrlr.function 
    cparams = ctrlr.params
    fx, gx = cfunc(q, env, xstar, cparams)
    return [q[2], q[3], fx[0], fx[1], gx[0], gx[1]]

goal = np.array([-7.0, -4.])
gains = (1., 2., 1.) #kappa, kg, eta
ctrlr = Controller(govController, goal, gains)
world = CircleWorld(np.array([0.,0.]), 7.0)
robot = Robot()
sim = Simulation(world, robot, ctrlr, dynamics)

q0 = [4.,3.5,0.,0.,4.,3.5]
tStop = 2000
tInc = 0.05
sim.run(q0, tStop, tInc)
sim.show()


def sensorBoundary(x, world, theta):
    if (world.type == 'circle'):
        b = 2. * (np.cos(theta) * x[0] + np.sin(theta) * x[1])
        c = -world.radius**2 + np.linalg.norm(x)**2
        rho = (-b + np.sqrt(b**2 - 4. * c)) / 2.
        return rho
    elif(world.type == 'rect'):
        tr = np.array([ world.length / 2. - x[0], world.width /2.  - x[1]])
        tl = np.array([-world.length / 2. - x[0], world.width /2.  - x[1]])
        bl = np.array([-world.length / 2. - x[0],-world.width /2.  - x[1]])
        br = np.array([ world.length / 2. - x[0],-world.width /2.  - x[1]])
        corners = [tr, tl, bl, br]
        cthetas = [np.arctan2(tr[1],tr[0]), np.arctan2(tl[1],tl[0]), np.arctan2(bl[1],bl[0]), np.arctan2(br[1],br[0])]
        cthetas = [np.pi *2 + x if x < 0. else x for x in cthetas]
        if(theta in cthetas):
            return np.linalg.norm(corners[cthetas.index(theta)])
        bisect.insort(cthetas, theta)
        print(cthetas)
        if(cthetas.index(theta)==0 or cthetas.index(theta)==4 ):
            return np.abs(world.length / 2. - x[0]) / np.cos(theta)
        elif(cthetas.index(theta)==1):
            return np.abs( world.width /2.  - x[1]) / np.sin(theta)
        elif(cthetas.index(theta)==2):
            return np.abs(-world.length / 2. - x[0]) / np.cos(theta)
        elif(cthetas.index(theta)==3):
            return np.abs(-world.width /2.  - x[1]) / np.sin(theta)

def sensorObstacle(x, world, theta):
    rho = np.inf
    for ob in world.obstacles:
        b = 2. * (np.cos(theta) * (x[0] - ob.center[0]) + np.sin(theta) * (x[1] - ob.center[1]))
        c = -ob.radius**2 + np.linalg.norm(x - ob.center)**2
        try:
            rhon = (-b - np.sqrt(b**2 - 4. * c)) / 2.
            if(rhon < rho):
                rho = rhon
        except:
            pass
    return rho 

def sensor(x, world, theta):
    return min(sensorObstacle(x,world,theta), sensorBoundary(x,world,theta))
