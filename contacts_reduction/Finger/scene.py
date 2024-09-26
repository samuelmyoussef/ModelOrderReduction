import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime

from os.path import abspath, dirname

path = dirname(abspath(__file__))


mesh_file = "/mesh/fingerFine"

RID = [
0,
82,
101,
102,
133,
253,
265,
274,
275,
276,
315,
332,
337,
358,
367,
371,
]

surface_triangles = [3486, 1124, 3423, 3463]

vertices = [[1605, 1606, 1604], [535, 536, 511], [1572, 1579, 1570], [1601, 1594, 1589]]


def createScene(root):
    plugins_list = ["SofaPython3","SoftRobots","ModelOrderReduction","STLIB",
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
         'Sofa.Component.Topology.Container.Dynamic', # Needed to use components [TetrahedronSetTopologyContainer]
         "Sofa.Component.Topology.Container.Constant",
         "Sofa.Component.Topology.Container.grid",
         "Sofa.Component.Topology.Mapping",
         "Sofa.Component.Topology.Container.Dynamic",
         "Sofa.Component.Collision.Geometry"]
    
    plugins = root.addChild("Plugins")
    for name in plugins_list:
        plugins.addObject("RequiredPlugin", name=name, printLog=False)
    
    root.addObject('VisualStyle', displayFlags='showVisualModels showCollisionModels')

    root.gravity = [0,0,0]
    root.dt = 0.025

    root.addObject('DefaultAnimationLoop')
    
    sphereTranslation = [0,10,0]
    
    loader = root.addObject('MeshVTKLoader', name='loader', filename=path + mesh_file + ".vtk", translation=sphereTranslation)
    # tetra_container = root.addObject('TetrahedronSetTopologyContainer', src='@loader', name='Container')
    tetra_container = root.addObject('TetrahedronSetTopologyContainer', position=loader.position.getLinkPath(), triangles=loader.triangles.getLinkPath(), tetrahedra=loader.tetrahedra.getLinkPath(), name='Container')
    root.addObject('TetrahedronSetTopologyModifier', name='Modifier', removeIsolated=False)
    # tri_container = root.addObject('TriangleSetTopologyContainer', name='TriContainer', src='@loader')
    # root.addObject('TriangleSetTopologyModifier', name='TriModifier')
    # root.addObject('TriangleSetGeometryAlgorithms', name='GeomAlgo')
    root.addObject('MechanicalObject', name='_mecha', template="Vec3d")
    root.addObject('UniformMass', name='mass', totalMass=0.2)
    # root.addObject('TetrahedronFEMForceField', name='FEM', youngModulus=40, poissonRatio=0.3)


    topology = root.addObject('MeshTopology',
                              name='topo',
                              position=tetra_container.position.linkpath,
                              # edges=tetra_container.edges.linkpath,
                              triangles=tetra_container.triangles.linkpath,
                              tetrahedra=tetra_container.tetrahedra.linkpath)


    # collision = root.addChild('Collision')
    # collision.addObject('MeshSTLLoader', name='loader', filename=path + mesh_file + ".stl",  rotation="0 0 0", translation=sphereTranslation)
    # collision.addObject('TriangleSetTopologyContainer', name='Container', src='@loader', tags='LDI')
    # collision.addObject('TriangleSetTopologyModifier', name='Modifier')
    # collision.addObject('MechanicalObject', position="@topology.position", template="Vec3d", name="coll")
    # collision.addObject('BarycentricMapping')
    # collision.addObject('PointCollisionModel')
    # collision.addObject('LineCollisionModel')
    # collision.addObject('TriangleCollisionModel')
    
    
    subset_topology = root.addChild('SubsetTopology')
    subset_topology.addObject('TriangleSetTopologyContainer', name='SubsetContainer', position="@../loader.position", triangles=vertices)
    subset_topology.addObject('TriangleSetTopologyModifier', name='Modifier')
    subset_topology.addObject('TriangleSetGeometryAlgorithms', name='GeomAlgo')
    subset_topology.addObject('SubsetTopologicalMapping', input="@topo", output="@SubsetContainer", samePoints="true")
    subset_topology.addObject('MechanicalObject', name="subset_mech")
    subset_topology.addObject('IdentityMapping')
    subset_topology.addObject('TriangleCollisionModel', color=[1, 0, 0, 1], group="1", printLog="1")


    visu = root.addChild('visual')
    visu.addObject('MeshSTLLoader', name='loader', filename=path + "/mesh/finger.stl", translation=sphereTranslation)
    visu.addObject('MeshTopology', src=visu.loader.getLinkPath(), name='topology')
    visu.addObject('OglModel', name='visual', src='@topology', cullFace='0', depthTest=True, scaleTex='100 100')
    visu.addObject('BarycentricMapping')

    return root
