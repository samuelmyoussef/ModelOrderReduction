import Sofa
import Sofa.Core
import os
pathSceneFile = os.path.dirname(os.path.abspath(__file__))
pathMesh = os.path.dirname(os.path.abspath(__file__))+'/'
nbrOfModes = 4
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

def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName=plugins, printLog=False)

                rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
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
                model.addObject('MeshVTKLoader', name='loader', filename=pathMesh+'sphere.vtk', translation=sphereTranslation)
                #model.addObject('Mesh',src = '@loader')
                Container = model.addObject('TetrahedronSetTopologyContainer',src = '@loader')
                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0',printLog="0")
#                model.addObject('WriteState', name="StateWriter", filename="fullBall.state",period="0.001", writeX=True, writeX0=True, writeV=False, writeF=False, time=0)
                model.addObject('UniformMass', totalMass='0.2', printLog='0')
#                model.addObject('TetrahedronFEMForceField', template='Vec3d',youngModulus=1.0e3,poissonRatio=0.45)
#                model.addObject('HyperReducedTetrahedronFEMForceField' , template = 'Vec3', method = 'large', name = 'reducedFF_modelNode_0', poissonRatio = 0.45, youngModulus = 1.0e3, nbModes = nbrOfModes, prepareECSW = True, performECSW = False, modesPath = 'modes.txt',nbTrainingSet=100,periodSaveGIE=10)
                model.addObject('HyperReducedTetrahedronFEMForceField' , template = 'Vec3', method = 'large', name = 'reducedFF_modelNode_0', poissonRatio = 0.45, youngModulus = 1.0e3, nbModes = nbrOfModes, prepareECSW = False, performECSW = True, modesPath = 'modes.txt', RIDPath='RID.txt',weightsPath='weights.txt')

                model.addObject('ModelOrderReductionMapping' , input = '@../modes', modesPath = 'modes.txt', output = '@./tetras')

                # modelCollis = model.addChild('modelCollis')
                # modelCollis.addObject('MeshSTLLoader', name='loader', filename=pathMesh+'sphere.stl', rotation="0 0 0", translation=sphereTranslation)
                # myContainer = modelCollis.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
                # modelCollis.addObject('MechanicalObject', name='collisMO', template='Vec3d')
                # modelCollis.addObject('BarycentricMapping')


                positions = Container.position.value
                edges = Container.edges.value
                triangles = Container.triangles.value

                ### List of triangles to keep in the collision model ###
                listElem = [278, 222, 262, 97, 224, 127]
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

                modelSubCollis = model.addChild('modelSubCollis')
                modelSubCollis.addObject('TriangleSetTopologyContainer',  name='subContainer', position=subPositions, triangles=subTriangles)
                modelSubCollis.addObject('MechanicalObject', name='subCollisMO', template='Vec3d')
                modelSubCollis.addObject('TriangleCollisionModel',group="0")
                modelSubCollis.addObject('LineCollisionModel',group="0")
                modelSubCollis.addObject('PointCollisionModel',group="0")
                modelSubCollis.addObject('BarycentricMapping')




                ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = model.addChild('visu')
                modelVisu.addObject('MeshSTLLoader', name='loader', filename=pathMesh+'sphere.stl', rotation="0 0 0", translation=sphereTranslation)
                modelVisu.addObject('OglModel', src='@loader', template='Vec3d', color='0.7 0.7 0.7 0.6')
                modelVisu.addObject('BarycentricMapping')

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
