#!/usr/bin/python3 -B

'''
PyDrake simulation of a 3D with a
LQR controller and energy shaping controller
and a state machine

meant to be foundation to a simplified
furuta pendulum simulator and controller

based off of cartpole_lqr.py && rx_the_simple_pendulum.py
'''

import numpy as np
import meshcat
import argparse
import matplotlib.pyplot as plt
from copy import copy

from pydrake.all import (Saturation, SignalLogger, wrap_to, VectorSystem)

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
 - try a different visualizer (as seen in simple pendulum.py) than PoseBundle in cartpole_lqr.py
   ex:

/   # Setup visualization
/   scene_graph = builder.AddSystem(SceneGraph())
/   PendulumGeometry.AddToBuilder(builder, pendulum.get_state_output_port(), scene_graph)
/   visualizer = pydrake.systems.meshcat_visualizer.ConnectMeshcatVisualizer(
/       builder,
/       scene_graph=scene_graph,
/       zmq_url=zmq_url)
/   visualizer.set_planar_viewpoint()
/   visualizer.vis.delete()


'''


# Energy Shaping Controller
'''
 simply the energy shaping controller, with option to plot the
 closed-loop phase portrait.
 NOTE: system is not actually stable @ upright, only attractive
'''
class EnergyShapingCtrlr(VectorSystem):

    def __init__(self, cart_pole):
        VectorSystem.__init__(self, 2, 1) # TODO: WHY, DOES THIS ALSO WORK FOR CARTPOLE? IS THIS U??
        self.cart_pole = cart_pole
        self.cart_pole_context = cart_pole.CreateDefaultContext()
        self.set_cart_pole_params()
