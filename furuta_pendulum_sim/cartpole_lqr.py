#!/usr/bin/python3 -B

'''
PyDrake simulation of a 3D with a
LQR controller

meant to be foundation to a simplified
furuta pendulum simulator and controller

based off of cartpole_lqr.py && rx_the_simple_pendulum.py &&
pydrake bindings example cart_pole_passive_simulation.py

state vector: [x, theta, xdot, thetadot]
'''

import numpy as np
import meshcat
import argparse
import matplotlib.pyplot as plt
from copy import copy

from pydrake.all import (Saturation, SignalLogger, wrap_to, VectorSystem, Linearize)

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




def main():

    sdf_path = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf")

    builder = DiagramBuilder()
    cart_pole = builder.AddSystem(MultibodyPlant(time_step=0.)) # TODO: time_step=args.time_step
    scene_graph = builder.AddSystem(SceneGraph()) # visualization & collision checking tool
    cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(plant=cart_pole).AddModelFromFile(sdf_path)

    # users must call Finalize() after making any additions to the multibody plant and
    # before using this class in the Systems framework, e.g. diagram = builder.Build()
    cart_pole.Finalize()
    assert cart_pole.geometry_source_is_registered() # NOTE: don't know if need this

    # wire up scene_graph and cart_pole geometry
    builder.Connect(
        scene_graph.get_query_output_port(),
        cart_pole.get_geometry_query_input_port())
    builder.Connect(
        cart_pole.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(cart_pole.get_source_id()))

    # hookup //tools:drake_visualizer
    DrakeVisualizer.AddToBuilder(builder=builder,
                                 scene_graph=scene_graph)

    # ----------------------------------------------------------
    # cartpole actuation (u) input port
    input_i = cart_pole.get_actuation_input_port().get_index()
    # cartpole state (x) output port
    output_i = cart_pole.get_state_output_port().get_index()

    # set the ctrlr included cart_pole context
    cart_pole_context_ = cart_pole.CreateDefaultContext()

    # set the fixed point to linearize around in lqr
    cart_pole_context_.get_mutable_continuous_state_vector().SetFromVector([0, np.pi, 0, 0])

    cart_pole.get_actuation_input_port().FixValue(cart_pole_context_, [0])

    # create lqr controller
    lqr = LinearQuadraticRegulator(cart_pole, cart_pole_context_,
                                   Q=np.eye(4), # large Q->stabilize sys w/ least poss. changes in states
                                   R=np.eye(1) * 0.1, # small R->cheap ctrl strat (no penalize ctrl signal)
                                   input_port_index=int(input_i))
    lqr = builder.AddSystem(lqr)
    # connect/wire up the lqr controller
    builder.Connect(cart_pole.get_state_output_port(),
                    lqr.get_input_port(0))
    builder.Connect(lqr.get_output_port(0),
                    cart_pole.get_actuation_input_port())
    # ----------------------------------------------------------



    diagram = builder.Build() # done defining & hooking up the system
    diagram_context = diagram.CreateDefaultContext()

    # cart_pole context based on controller-less diagram context
    #cart_pole_context = diagram.GetMutableSubsystemContext(cart_pole,
    #                                                       diagram_context)
    # NOTE: uncomment to set actuation to this context w/o lqr
    #cart_pole.get_actuation_input_port().FixValue(cart_pole_context, [0])


    # instantiate a simulator
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False) # speed up sim
    simulator.set_target_realtime_rate(1.0)

    # sim context, reset initial time & state
    sim_context = simulator.get_mutable_context()
    sim_context.SetTime(0.)
    sim_context.SetContinuousState([0, np.pi + 0.3, 0, 0])

    ''' NOTE: another way to set cartpole state
    cart_slider = cart_pole.GetJointByName("CartSlider")
    pole_pin = cart_pole.GetJointByName("PolePin")
    cart_slider.set_translation(context=cart_pole_context, translation=0.)
    pole_pin.set_angle(context=cart_pole_context, angle=2.)
    '''

    # run sim until simulator.AdvanceTo(n) seconds
    simulator.Initialize()
    simulator.AdvanceTo(15.0)



    exit(0)






if __name__ == "__main__":
    main()
