# -*- coding: utf-8 -*-

import Sofa
import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

from stlib3.scene import Scene
# A prefab that implements an ElasticMaterialObject
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel


mesh_file = "finger"


class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs['rootNode']
        self.cableConstraint = kwargs['cableConstraint']
        # self.cable = self.rootNode.solverNode.finger.cable.cable.getData('value')
        self.cable = self.cableConstraint.getData('value')

    def onKeypressedEvent(self, event):
        key = event['key']

        if key == "A":
            self.cable.value = [self.cable.value[0] + 1.]

        elif key == "M":
            displacement = self.cable.value[0] - 1.
            if displacement < 0:
                displacement = 0.
            self.cable.value = [displacement]




def createScene(rootNode):
    rootNode.addObject("RequiredPlugin",
                       pluginName=["SoftRobots",
                        'Sofa.Component.AnimationLoop',
                        'Sofa.Component.Constraint.Lagrangian.Correction',
                        'Sofa.Component.Constraint.Lagrangian.Solver',
                        'Sofa.Component.Engine.Select',
                        'Sofa.Component.IO.Mesh',
                        'Sofa.Component.LinearSolver.Direct',
                        'Sofa.Component.Mapping.Linear',
                        'Sofa.Component.Mass',
                        'Sofa.Component.ODESolver.Backward',
                        'Sofa.Component.SolidMechanics.FEM.Elastic',
                        'Sofa.Component.SolidMechanics.Spring',
                        'Sofa.Component.StateContainer',
                        'Sofa.Component.Topology.Container.Dynamic',
                        'Sofa.Component.Visual',
                        'ModelOrderReduction'])


    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0], dt=0.01)
    scene.VisualStyle.displayFlags='showBehavior showCollisionModels'
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', name='GSSolver', maxIterations='10000', tolerance='1e-15')
    rootNode.addObject('CollisionPipeline', verbose="0")
    rootNode.addObject('BruteForceBroadPhase', name="N2")
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response="FrictionContactConstraint", responseParams="mu=0.0")
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance="8.0", contactDistance="0.5", angleCone="0.01")

    solverNode = rootNode.addChild('solverNode')
    solverNode.addObject('EulerImplicitSolver', rayleighStiffness = 0.0, rayleighMass = 0.0)
    solverNode.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")
    solverNode.addObject('GenericConstraintCorrection')


    cube = rootNode.addChild('cube')
    cube.addObject('MeshOBJLoader', name="loader", filename="mesh/smCube27.obj",scale="3.0", translation=[-80,40,7.5])
    cube.addObject('TriangleSetTopologyContainer', src='@loader', name='container')

    cube.addObject('MechanicalObject', name='collisMO', template='Vec3d')
    cube.addObject('TriangleCollisionModel',group="0")
    cube.addObject('LineCollisionModel',group="0")
    cube.addObject('PointCollisionModel',group="0")
    cube.addObject('UncoupledConstraintCorrection')

    # cube.addObject('PartialFixedProjectiveConstraint', indices=list(range(0, 27)), fixedDirections=[0, 1, 1])
    # cube.addObject('PositionConstraint', indices=list(range(0, 27)),
    #                valueType="displacement", useDirections=[1, 0, 0])

    actuator  = rootNode.addChild('actuator')
    actuator.addObject('MechanicalObject', name='cubeactuator', position='@cube/collisMO.position', template='Vec3d')
    
    finger = solverNode.addChild('finger')
    finger.addObject('MeshVTKLoader', name="loader", filename=path+mesh_file+'.vtk')
    finger.addObject('TetrahedronSetTopologyContainer', position="@loader.position", tetrahedra="@loader.tetrahedra")
    finger.addObject('MechanicalObject')
    finger.addObject('UniformMass', totalMass=0.05)
    finger.addObject('TetrahedronFEMForceField', poissonRatio="0.45", youngModulus="600")

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use "BoxROI"...
    finger.addObject('BoxROI', name='ROI1', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    # Or "SphereROI"...
    # finger.addObject('SphereROI', name='ROI', centers='0 0 0', radii='5')

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)

    modelCollis = finger.addChild('modelCollis')
    modelCollis.addObject('MeshSTLLoader', name='loader', filename=path+mesh_file+'.stl')
    modelCollis.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
    modelCollis.addObject('MechanicalObject', name='collisMO', template='Vec3d')
    modelCollis.addObject('TriangleCollisionModel',group="1")
    modelCollis.addObject('LineCollisionModel',group="1")
    modelCollis.addObject('PointCollisionModel',group="1")
    modelCollis.addObject('BarycentricMapping')


    ##########################################
    # Visualization                          #
    ##########################################

    modelVisu = finger.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', name='loader', filename=path+mesh_file+'.stl')
    modelVisu.addObject('OglModel', src='@loader', template='Vec3d', color='0.7 0.7 0.7 0.6')
    modelVisu.addObject('BarycentricMapping')



    ##########################################
    # Cable                                  #
    ##########################################

    #  This adds a new node in the scene. This node is appended to the finger's node.
    cable = finger.addChild('cable')

    # This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.addObject('MechanicalObject',
                    position=[
                        [-17.5, 12.5, 2.5],
                        [-32.5, 12.5, 2.5],
                        [-47.5, 12.5, 2.5],
                        [-62.5, 12.5, 2.5],
                        [-77.5, 12.5, 2.5],

                        [-83.5, 12.5, 4.5],
                        [-85.5, 12.5, 6.5],
                        [-85.5, 12.5, 8.5],
                        [-83.5, 12.5, 10.5],

                        [-77.5, 12.5, 12.5],
                        [-62.5, 12.5, 12.5],
                        [-47.5, 12.5, 12.5],
                        [-32.5, 12.5, 12.5],
                        [-17.5, 12.5, 12.5]])

    # add a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    cableConstraint = cable.addObject('CableConstraint', name="cable",
                    indices=list(range(0, 14)),
                    pullPoint=[0.0, 12.5, 2.5])

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bidirectional link
    # between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.addObject('BarycentricMapping')



    rootNode.addObject(FingerController(name="FingerController", rootNode=rootNode, cableConstraint=cableConstraint))



    return rootNode
