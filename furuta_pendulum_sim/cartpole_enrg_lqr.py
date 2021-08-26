#!/usr/bin/python3 -B

'''
PyDrake simulation of a 3D with a
LQR controller and energy shaping controller
and a state machine

meant to be foundation to a simplified
furuta pendulum simulator and controller

based off of cartpole_lqr.py && rx_the_simple_pendulum.py

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


'''
TODO:
 [ ] try a different visualizer (as seen in simple pendulum.py) than PoseBundle in cartpole_lqr.py
     ex:

/    # Setup visualization
/    scene_graph = builder.AddSystem(SceneGraph())
/    PendulumGeometry.AddToBuilder(builder, pendulum.get_state_output_port(), scene_graph)
/    visualizer = pydrake.systems.meshcat_visualizer.ConnectMeshcatVisualizer(
/        builder,
/        scene_graph=scene_graph,
/        zmq_url=zmq_url)
/    visualizer.set_planar_viewpoint()
/    visualizer.vis.delete()

 [ ] test energy shaping controller
 [ ] test lqr controller
 [ ] test energy shaping + lqr controller

'''


# Energy Shaping Controller
'''
 simply the energy shaping controller, with option to plot the
 closed-loop phase portrait.
 NOTE: system is not actually stable @ upright, only attractive
'''
class EnergyShapingCtrlr(VectorSystem):

    def __init__(self, cart_pole, x_star):
        # VectorSystem: A base class that specializes LeafSystem
        # for use with only zero or one vector input ports, and
        # only zero or one vector output ports.
        VectorSystem.__init__(self, 4, 1) # input size 4, output size 1
        self.cart_pole = cart_pole
        self.cart_pole_context = cart_pole.CreateDefaultContext()
        self.SetCartpoleParams(x_star)

    def SetCartpoleParams(self, x_star):
        # setting params
        self.cart_pole_context.get_mutable_continuous_state_vector().SetFromVector(x_star)
        # upright state w/ cart back at origin
        self.cart_pole_context.SetContinuousState([0, np.pi, 0, 0])
        # energy at upright state w/ cart back at origin
        self.desired_energy = self.cart_pole.EvalPotentialEnergy(self.cart_pole_context)

    # actual controller u (xdotdot in this case)
    def DoCalcVectorOutput(self, context, cart_pole_state, unused, output):
        # set context for current cartpole state
        self.cart_pole_context.SetContinuousState(cart_pole_state)
        # get cartpole params
        params = self.cart_pole_context.get_numeric_parameter(0)
        k_E = 0.1 # feedback controller gain for energy
        k_p = 0.1 # proportional gain to regulate cart
        k_d = 0.1 # derivative gain to regulate cart
        x = cart_pole_state[0]
        theta = cart_pole_state[1]
        xdot = cart_pole_state[2]
        thetadot = cart_pole_state[3]
        total_energy = (self.cart_pole.EvalPotentialEnergy(self.cart_pole_context) +
                        self.cart_pole.EvalKineticEnergy(self.cart_pole_context))
        # This is the energy shaping ctrlr for cartpole, which is essentially the
        # simple pendulum energy ctrlr (with cos term due to collocated PFL) and
        # a PD controller to regulate the cart
        output[:] = (k_E * thetadot * np.cos(theta) * (total_energy - self.desired_energy) -
                     k_p * x -
                     k_d * xdot +
                     params.damping() * thetadot) # damping term


# LQR controller class (to better organize and consolidate the weight matrices, Q, R)
# NOTE: might be better for future organization to make this into a standalone function
#       but I just want to experiment here
class BalancingLQRCtrlr():

    # TODO: need input_port, and output_port from
    #       input_i = cart_pole.get_actuation_input_port().get_index()
    #       output_i = cart_pole.get_state_output_port().get_index
    def __init__(self, cart_pole, input_i, output_i, Q, R):
        self.cart_pole = cart_pole
        self.Q = Q
        self.R = R
        self.context = cart_pole.CreateDefaultContext()
        self.cart_pole.get_actuation_input_port().FixValue(self.context, [0])
        self.context.SetContinuousState([0, np.pi, 0, 0]) # upright pos w/ cart @ origin
        # NOTE: might need to get the i/o ports for the systems to linearize
        self.linearized_cart_pole = Linearize(self.cart_pole, self.context,
                                              input_port_index=input_i,
                                              output_port_index=output_i)

    # lqr controller matrices (K ctrlr matrix, S Riccati eq matrix (used in optimal cost-to-go fxn))
    def BalancingLQR(self):
        (K, S) = LinearQuadraticRegulator(self.linearized_cart_pole.A(),
                                          self.linearized_cart_pole.B(), self.Q, self.R)
        return (K, S)



# Combined Energy Shaping (SwingUp) and LQR (Balance) Controller
# with a simple state machine
class SwingUpAndBalanceController(VectorSystem):

    def __init__(self, cart_pole):
        VectorSystem.__init__(self, 4, 1)






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

    # cartpole actuation (u) input port
    input_i = cart_pole.get_actuation_input_port().get_index()
    # cartpole state (x) output port
    output_i = cart_pole.get_state_output_port().get_index()

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

    diagram = builder.Build() # done defining & hooking up the system


    # create passive cartpole (no ctrlr) diagram_context & cartpole context
    diagram_context = diagram.CreateDefaultContext()
    cart_pole_context_passive = diagram.GetMutableSubsystemContext(cart_pole,
                                                           diagram_context)
    # set actuation port based on no controller cart_pole_context_passive
    cart_pole.get_actuation_input_port().FixValue(cart_pole_context_passive, 0)

    # instantiate a simulator
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False) # speed up sim
    simulator.set_target_realtime_rate(1.0)

    # sim context, reset initial time & state
    sim_context = simulator.get_mutable_context()
    sim_context.SetTime(0.)
    sim_context.SetContinuousState([0, 0.1, 0, 0])

    ''' NOTE: another way to set cartpole state
    cart_slider = cart_pole.GetJointByName("CartSlider")
    pole_pin = cart_pole.GetJointByName("PolePin")
    cart_slider.set_translation(context=cart_pole_context, translation=0.)
    pole_pin.set_angle(context=cart_pole_context, angle=2.)
    '''

    # run sim until simulator.AdvanceTo(n) seconds
    simulator.Initialize()
    simulator.AdvanceTo(10.0)



    exit(0)






if __name__ == "__main__":
    main()
