import Sofa
import Sofa.Core
import os
pathSceneFile = os.path.dirname(os.path.abspath(__file__))
pathMesh = os.path.dirname(os.path.abspath(__file__))+'/'
nbrOfModes = 15
# Units: mm, kg, s.     Pressure in kPa = k (kg/(m.s^2)) = k (g/(mm.s^2) =  kg/(mm.s^2)
plugins=["SofaPython3","SoftRobots","ModelOrderReduction","STLIB",
         # normally plugin Sofa.Component is enough but still warning
         "Sofa.Component.Visual",
         "Sofa.Component.AnimationLoop",
         "Sofa.GL.Component.Rendering3D",
         "Sofa.Component.Constraint.Lagrangian.Solver",
         'Sofa.Component.IO.Mesh',
         'Sofa.Component.Playback',
         'Sofa.Component.Constraint.Lagrangian.Correction', # Needed to use components [GenericConstraintCorrection]
         'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
         'Sofa.Component.LinearSolver.Direct', # Needed to use components [SparseLDLSolver]
         'Sofa.Component.Mapping.Linear', # Needed to use components [BarycentricMapping]
         'Sofa.Component.Mass', # Needed to use components [UniformMass]
         'Sofa.Component.ODESolver.Backward', # Needed to use components [EulerImplicitSolver]
         'Sofa.Component.SolidMechanics.FEM.Elastic', # Needed to use components [TetrahedronFEMForceField]
         'Sofa.Component.SolidMechanics.Spring', # Needed to use components [RestShapeSpringsForceField]
         'Sofa.Component.StateContainer', # Needed to use components [MechanicalObject]
         'Sofa.Component.Topology.Container.Dynamic'] # Needed to use components [TetrahedronSetTopologyContainer]

surface_triangles = [289, 617, 672, 1034, 1402, 1449]
vertices = [[66, 77, 133], [52, 104, 131], [62, 137, 71], [23, 131, 27], [53, 56, 142], [29, 137, 105]]


def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName=plugins, printLog=False)

                rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields hideWireframe')
                rootNode.gravity=[0,-9810, 0]
                rootNode.dt = 0.0001
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', name='GSSolver', maxIterations='10000', tolerance='1e-15')
                rootNode.addObject('CollisionPipeline', verbose="0")
                rootNode.addObject('BruteForceBroadPhase', name="N2")
                rootNode.addObject('BVHNarrowPhase')
                rootNode.addObject('CollisionResponse', response="FrictionContactConstraint", responseParams="mu=0.0")
                rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance="8.0", contactDistance="0.5", angleCone="0.01")

                solverNode = rootNode.addChild('solverNode')
                solverNode.addObject('EulerImplicitSolver', name='odesolver',firstOrder="false", rayleighStiffness='0.0', rayleighMass='0.0')
                solverNode.addObject('SparseLDLSolver', name="preconditioner", template="CompressedRowSparseMatrixd")
                solverNode.addObject('GenericConstraintCorrection', linearSolver='@preconditioner',printLog=True, name='ResReso')

                ##########################################
                # FEM Model                              #
                ##########################################
                
                sphereTranslation = [0,10,0]
                modesNode = solverNode.addChild('modesNode')
                modesNode.addObject('MechanicalObject', position = [0]*nbrOfModes, name='modes', template='Vec1d', showIndices='false')

                model = modesNode.addChild('model')
                loader = model.addObject('MeshVTKLoader', name='loader', filename=pathMesh+'sphere.vtk', translation=sphereTranslation)
                # model.addObject('Mesh',src = '@loader')
                tetra_container = model.addObject('TetrahedronSetTopologyContainer', position=loader.position.getLinkPath(), tetrahedra=loader.tetrahedra.getLinkPath(), name='Container')
                model.addObject('TetrahedronSetTopologyModifier', name='Modifier', removeIsolated=False)
                # tri_container = model.addObject('TriangleSetTopologyContainer', name='TriContainer', src='@loader')
                # model.addObject('TriangleSetTopologyModifier', name='TriModifier')
                # model.addObject('TriangleSetGeometryAlgorithms', name='GeomAlgo')
                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0',printLog="0")
#               model.addObject('WriteState', name="StateWriter", filename="fullBall.state",period="0.001", writeX=True, writeX0=True, writeV=False, writeF=False, time=0)
                model.addObject('UniformMass', totalMass='0.2', printLog='0', name='mass')
#               model.addObject('TetrahedronFEMForceField', template='Vec3d',youngModulus=1.0e3,poissonRatio=0.45)
#               model.addObject('HyperReducedTetrahedronFEMForceField' , template = 'Vec3', method = 'large', name = 'reducedFF_modelNode_0', poissonRatio = 0.45, youngModulus = 1.0e3, nbModes = nbrOfModes, prepareECSW = True, performECSW = False, modesPath = 'modes.txt',nbTrainingSet=100,periodSaveGIE=10)
                model.addObject('HyperReducedTetrahedronFEMForceField' , template = 'Vec3', method = 'large', name = 'reducedFF_modelNode_0', poissonRatio = 0.45, youngModulus = 1.0e3, nbModes = nbrOfModes, prepareECSW = False, performECSW = True, modesPath = 'modes.txt', RIDPath='RID.txt',weightsPath='weights.txt')

                model.addObject('ModelOrderReductionMapping' , input = '@../modes', modesPath = 'modes.txt', output = '@./tetras')

                topology = rootNode.addObject('MeshTopology',
                                            name='topo',
                                            position=tetra_container.edges.linkpath,
                                            edges=tetra_container.edges.linkpath,
                                            triangles=tetra_container.triangles.linkpath,
                                            tetrahedra=tetra_container.tetrahedra.linkpath)


                modelCollis = model.addChild('modelCollis')
                # modelCollis.addObject('MeshSTLLoader', name='loader', filename=pathMesh+'sphere.stl', rotation="0 0 0", translation=sphereTranslation)
                # modelCollis.addObject('TriangleSetTopologyContainer', name='Container', src='@loader', tags='LDI')
                # modelCollis.addObject('TriangleSetTopologyModifier', name='Modifier')
                # modelCollis.addObject('MechanicalObject', template="Vec3d", name="coll")
                # modelCollis.addObject('TriangleCollisionModel',group="0")
                # modelCollis.addObject('LineCollisionModel',group="0")
                # modelCollis.addObject('PointCollisionModel',group="0")
                # modelCollis.addObject('BarycentricMapping')

                subset_topology = modelCollis.addChild('SubsetTopology')
                mesh_subset = subset_topology.addObject('MeshSubsetEngine', name='MeshSubset', inputPosition="@../../loader.position", indices=vertices)
                # print("DEBUG    -----------------------------", mesh_subset.position.value)
                positions = mesh_subset.position.value
                subset_topology.addObject('TriangleSetTopologyContainer', name='SubsetContainer', position=positions, triangles=vertices)
                # subset_topology.addObject('TriangleSetTopologyContainer', name='SubsetContainer', position="@../../loader.position", triangles=vertices)
                subset_topology.addObject('TriangleSetTopologyModifier', name='Modifier')
                subset_topology.addObject('TriangleSetGeometryAlgorithms', name='GeomAlgo')
                # subset_topology.addObject('SubsetTopologicalMapping', input="@../../Container", output="@SubsetContainer", samePoints="true", handleEdges="false", handleTriangles="true")
                subset_topology.addObject('MechanicalObject', name="subset_mech", position=positions)
                subset_topology.addObject('IdentityMapping')
                subset_topology.addObject('TriangleCollisionModel', color=[1, 0, 0, 1])
                subset_topology.addObject('LineCollisionModel')
                subset_topology.addObject('PointCollisionModel')

                
                # visu = model.addChild('visual')
                # visu.addObject('MeshVTKLoader', name='loader', filename=pathMesh + "/sphere.vtk", translation=sphereTranslation)
                # visu.addObject('MeshTopology', src=visu.loader.getLinkPath(), name='topology')
                # visu.addObject('OglModel', name='visual', src='@topology', cullFace='0', depthTest=True, scaleTex='100 100')
                # visu.addObject('BarycentricMapping')


                rotation=[0,0,0]
                #rotation=[30,0,0]
                planeNode = rootNode.addChild('Plane')
                planeNode.addObject('MeshObjLoader', name='loader', filename="mesh/floorFlat.obj", triangulate="true",rotation=rotation)
                planeNode.addObject('Mesh', src="@loader")
                planeNode.addObject('MechanicalObject', src="@loader", rotation="0 0 0", translation="0 0 0", scale="1")
                planeNode.addObject('TriangleCollisionModel',simulated="0", moving="0",group="1",name='TriPlane')
                planeNode.addObject('LineCollisionModel',simulated="0", moving="0",group="1")
                planeNode.addObject('PointCollisionModel',simulated="0", moving="0",group="1")
                planeNode.addObject('OglModel',name="Visual", fileMesh="mesh/floorFlat.obj", color="1 0 0 1",rotation=rotation, translation="0 0 0", scale="1")
                planeNode.addObject('UncoupledConstraintCorrection')

                return rootNode
