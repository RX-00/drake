#!/usr/bin/python3 -B

'''
PyDrake simulation of a simple 2D cartpole with a
LQR controller as seen in the underactuated robotics
exercise. This program is an exercise to translate
that notebook to a locally exec program

also used run_planar_scenegraph_visualizer.py as a
guide for how to get it to work locally

NOTE: DOES NOT WORK
'''

import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import pydrake.all
from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder,
                         LinearQuadraticRegulator, Parser,
                         PlanarSceneGraphVisualizer, Simulator, Linearize)
import pydrake.common
from pydrake.common import (FindResourceOrThrow, temp_directory)

# Setup matplotlib.
from IPython import get_ipython
if get_ipython() is not None: get_ipython().run_line_magic("matplotlib", "inline")
from IPython.display import display


'''
 simulates cartpole + lqr controller given initial state
 and params until the given final time. Returns the state
 sampled at the given tape_period

 x_star = unstable equilibrium point
 Q = penalize state var, large -> stabilize sys w/ least possible changes in states
                         small -> less concern about changes in states
 R = penalize ctrl signal, large -> stabilize sys w/ less (weight) energy (expensive ctrl strat)
                           small -> penalize the ctrl signal less (cheap ctrl strat, so you can use more)
'''
x_star = [0, np.pi, 0, 0]
Q = np.eye(4)
R = np.eye(1)

# construct the block diagram w/ cartpole in closed loop
# w/ the lqr controller

# start construction site of our block diagram
builder = DiagramBuilder()

# instantiate the cartpole and the SceneGraph
# `SceneGraph` is the visualization and collision checking tool
cartpole, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
urdf_path = FindResourceOrThrow(
    "drake/examples/multibody/cart_pole/cartpole.urdf")
Parser(cartpole).AddModelFromFile(urdf_path)
cartpole.Finalize()

# set the operating point (vertical/upright unstable equilibrium)
context = cartpole.CreateDefaultContext()
context.get_mutable_continuous_state_vector().SetFromVector(x_star)

# fix the input port to 0 and get its index for the lqr function
cartpole.get_actuation_input_port().FixValue(context, [0])
input_i = cartpole.get_actuation_input_port().get_index()

# create lqr ctrlr directly from the nonlinear sys and the
# operating point
lqr = LinearQuadraticRegulator(cartpole, context, Q, R, input_port_index=int(input_i))
lqr = builder.AddSystem(lqr)

# example of getting the output state index
#output_i = cartpole.get_state_output_port().get_index()
# example of linearizing around the unstable equilibrium point
#cartpole_lin = Linearize(cartpole, context, input_port_index=input_i, output_port_index=output_i)

# Wire up the cartpole and lqr ctrlr together
builder.Connect(cartpole.get_state_output_port(), lqr.get_input_port(0))
builder.Connect(lqr.get_output_port(0), cartpole.get_actuation_input_port())

# add a visualizer & wire it
visualizer = builder.AddSystem(
    PlanarSceneGraphVisualizer(
        scene_graph, xlim=[-5., 5.], ylim=[-5, 5], show=False))
builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))

# finish building the block diagram
diagram = builder.Build()

# instantiate a simulator
simulator = Simulator(diagram)
simulator.set_publish_every_time_step(False) # makes sim faster


'''
 execute the simulation given initial state (x0) and the
 simulation time, simulates the closed loop system and
 produces a video
'''
def simulate_and_animate(x0, sim_time=5):
    # start recording the video for the animation of the simulation
    visualizer.start_recording()

    # reset initial time & state
    context = simulator.get_mutable_context()
    context.SetTime(0.)
    context.SetContinuousState(x0)

    # run sim
    simulator.Initialize()
    simulator.AdvanceTo(sim_time)

    # stop video
    visualizer.stop_recording()

    # construct animation
    animation = visualizer.get_recording_as_animation()

    #display()
    animation.save("{}/pend_playback.mp4".format(temp_directory()), fps=30)

    visualizer.reset_recoding()


if __name__ == "__main__":
    print("Simulating cartpole...")
    x0 = [0, np.pi, 0, 0]
    simulate_and_animate(x0)
