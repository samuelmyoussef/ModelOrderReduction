# -*- coding: utf-8 -*-

import Sofa
import os

path = os.path.dirname(os.path.abspath(__file__))
path_mesh = path + '/mesh/'

mesh_file = "finger"

from stlib3.scene import Scene
# A prefab that implements an ElasticMaterialObject
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel


modesPath = path + "/reduced/data/modes.txt"
RIDPath = path + "/reduced/data/reducedFF_finger_0_RID.txt"
weightsPath = path + "/reduced/data/reducedFF_finger_0_weight.txt"


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



nbrOfModes = 7



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
    

    modesNode = solverNode.addChild('modesNode')
    modesNode.addObject('MechanicalObject', position = [0]*nbrOfModes, name='modes', template='Vec1d', showIndices='false')

    finger = modesNode.addChild('finger')

    finger.addObject('MeshVTKLoader', name="loader", filename=path_mesh+mesh_file+'.vtk')
    Container = finger.addObject('TetrahedronSetTopologyContainer', src = '@loader')
    finger.addObject('MechanicalObject', name='tetras')
    finger.addObject('UniformMass', totalMass=0.05)
    # finger.addObject('TetrahedronFEMForceField', poissonRatio="0.45", youngModulus="600")
    finger.addObject('HyperReducedTetrahedronFEMForceField' , template = 'Vec3', method = 'large', name = 'reducedFF_modelNode_0', poissonRatio = 0.45, youngModulus = 600, nbModes = nbrOfModes, prepareECSW = False, performECSW = True, modesPath = modesPath, RIDPath = RIDPath, weightsPath = weightsPath)

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use "BoxROI"...
    # finger.addObject('BoxROI', name='ROI1', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    # Or "SphereROI"...
    # finger.addObject('SphereROI', name='ROI', centers='0 0 0', radii='5')

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    # finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)
    # finger.addObject('HyperReducedRestShapeSpringsForceField' , points = '@ROI1.indices', stiffness = 1000000000000.0, name = 'reducedFF_finger_1', nbModes = nbrOfModes, performECSW = True, modesPath = modesPath, RIDPath = RIDPath, weightsPath = weightsPath)

    finger.addObject('ModelOrderReductionMapping' , input = '@../modes', modesPath = modesPath, output = '@./tetras')

    # modelCollis = finger.addChild('modelCollis')
    # modelCollis.addObject('MeshSTLLoader', name='loader', filename=path_mesh+mesh_file+'.stl')
    # modelCollis.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
    # modelCollis.addObject('MechanicalObject', name='collisMO', template='Vec3d')
    # modelCollis.addObject('TriangleCollisionModel',group="1")
    # modelCollis.addObject('LineCollisionModel',group="1")
    # modelCollis.addObject('PointCollisionModel',group="1")
    # modelCollis.addObject('BarycentricMapping')


    positions = Container.position.value
    edges = Container.edges.value
    triangles = Container.triangles.value

    ### List of triangles to keep in the collision model ###
    listElem = [142, 216, 269, 133, 135, 139]
    ########################################################

    subTriangles = []
    subVertices = []
    nbVertices = 0
    for el in listElem:
        if triangles[el][0] not in subVertices:
            subTriangles.append(nbVertices)
            nbVertices = nbVertices+1
            subVertices.append(triangles[el][0])
        else:
            subTriangles.append(subVertices.index(triangles[el][0]))

        if triangles[el][1] not in subVertices:
            subTriangles.append(nbVertices)
            nbVertices = nbVertices+1
            subVertices.append(triangles[el][1])
        else:
            subTriangles.append(subVertices.index(triangles[el][1]))

        if triangles[el][2] not in subVertices:
            subTriangles.append(nbVertices)
            nbVertices = nbVertices+1
            subVertices.append(triangles[el][2])
        else:
            subTriangles.append(subVertices.index(triangles[el][2]))

    subPositions = []
    for vertice in subVertices:
        subPositions.append(positions[vertice])

    modelSubCollis = finger.addChild('modelSubCollis')
    modelSubCollis.addObject('TriangleSetTopologyContainer',  name='subContainer', position=subPositions, triangles=subTriangles)
    modelSubCollis.addObject('MechanicalObject', name='subCollisMO', template='Vec3d')
    modelSubCollis.addObject('TriangleCollisionModel',group="1")
    modelSubCollis.addObject('LineCollisionModel',group="1")
    modelSubCollis.addObject('PointCollisionModel',group="1")
    modelSubCollis.addObject('BarycentricMapping')


    ##########################################
    # Visualization                          #
    ##########################################

    modelVisu = finger.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', name='loader', filename=path_mesh + r'/finger.stl')
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
