#!/usr/bin/env python3

import pyexotica as exo
from pyexotica.publish_trajectory import *
from numpy import array
import math


def figure_eight(t):
    return array([0.7, math.sin(t * 2.0 * math.pi * 0.5) * 0.1, math.sin(t * math.pi * 0.5) * 0.2 + 0.5, 0.0, 0.0, 0.0])


exo.Setup.init_ros()
solver = exo.Setup.load_solver(
    '{motion_planning_exotica}/resources/configs/follow_traj.xml')
problem = solver.get_problem()
q = array([0.5,0.1,0.5,0,0,0])
problem.start_state = q

for t in range(0, problem.T):
    if t < problem.T/5:
        problem.set_rho('Frame', 0.0, t)
    else:
        problem.set_rho('Frame', 1e5, t)
        problem.set_goal('Frame', figure_eight(t*problem.tau), t)

solution = solver.solve()

#plot(solution)

publish_trajectory(solution, problem.T*problem.tau, problem)