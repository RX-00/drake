#!/usr/bin/python3 -B

'''
PyDrake simulation of a 3D with a
LQR controller

meant to be foundation to a simplified
furuta pendulum simulator and controller

NOTE: FUCKING WORKS
'''

import numpy as np
import argparse

from pydrake.common import FindResourceOrThrow
from pydrake.common import temp_directory
from pydrake.geometry import (DrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer, PlanarSceneGraphVisualizer)
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.all import Linearize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()

    sdf_path = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf")

    '''
     simulates cartpole + lqr controller given initial state
     and params until the given final time. Returns the state
     sampled at the given tape_period

     x_star = unstable equilibrium point
     Q = penalize state var, large -> stabilize sys w/ least possible changes in states
                         small -> less concern about changes in states
     R = penalize ctrl signal, large -> stabilize sys w/ less (weight) energy (expensive ctrl strat)
                           small -> penalize the ctrl signal less (cheap ctrl strat, so can use more)
    '''
    x_star = [0, np.pi, 0, 0]
    Q = np.eye(4)
    R = np.eye(1)

    # start construction site of our block diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph()) # visualization and collision checking tool
    cart_pole = builder.AddSystem(MultibodyPlant(time_step=args.time_step))
    cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(plant=cart_pole).AddModelFromFile(sdf_path)
    # users must call Finalize() after making any additions to the multibody plant and before using
    # this class in the Systems framework, e.g. diagram = builder.Build()
    cart_pole.Finalize()

    assert cart_pole.geometry_source_is_registered()

    # wire up scene to cart_pole geometry and vice versa
    builder.Connect(
        scene_graph.get_query_output_port(),
        cart_pole.get_geometry_query_input_port())
    builder.Connect(
        cart_pole.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(cart_pole.get_source_id()))

    cart_pole_context = cart_pole.CreateDefaultContext()
    # SETTING PARAMS
    cart_pole_context.get_mutable_continuous_state_vector().SetFromVector(x_star)

    cart_pole.get_actuation_input_port().FixValue(cart_pole_context, [0])
    input_i = cart_pole.get_actuation_input_port().get_index()
    lqr = LinearQuadraticRegulator(cart_pole, cart_pole_context, Q, R, input_port_index=int(input_i))

    # getting the K (ctrlr matrix) and S (Riccati eq matrix, used in optimal cost-to-go fxn) matrices
    output_i = cart_pole.get_state_output_port().get_index()
    lin_cart_pole = Linearize(cart_pole, cart_pole_context,
                              input_port_index=input_i, output_port_index=output_i)
    (K, S) = LinearQuadraticRegulator(lin_cart_pole.A(), lin_cart_pole.B(), Q, R)
    print("LQR RESULT: ")
    print(lqr)
    print(K)
    print(S)

    lqr = builder.AddSystem(lqr)
    # connect actuation
    builder.Connect(cart_pole.get_state_output_port(), lqr.get_input_port(0))
    builder.Connect(lqr.get_output_port(0), cart_pole.get_actuation_input_port())

    # add a visualizer & wire it
    visualizer = builder.AddSystem(
    PlanarSceneGraphVisualizer(
        scene_graph, xlim=[-3., 3.], ylim=[-1.2, 1.2], show=True))
    builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))

    diagram = builder.Build()

    x0 = [0, np.pi + 0.3, 0, 0]
    sim_time = 50

    # instantiate a simulator
    simulator = Simulator(diagram)
    simulator.set_publish_every_time_step(False) # makes sim faster

    # start recording the video for the animation of the simulation
    visualizer.start_recording()

    # reset initial time & state
    context = simulator.get_mutable_context()
    context.SetTime(0.)
    context.SetContinuousState(x0) # x, theta, xdot, thetadot

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
    main()
