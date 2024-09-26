import Sofa.Gui
import Sofa.Simulation
import importlib

from scene import createScene

RID = [
54,
108,
319,
515,
752,
1393,
1526,
1553,
2053,
2512,
2917,
2983,
3007,
3049,
3443,
3525,
3802,
3955,
4186,
4190,
4891,
5904,
11594,
11595,
]

def main():
    root = Sofa.Core.Node("root")

    createScene(root)
    Sofa.Simulation.init(root)

    total_triangles = []
    surface_triangles = []
    vertices = []

    topo_triangles = root.topo.triangles.value

    for tetrahedron in RID:
        tri_vertices = root.topo.tetrahedra[tetrahedron]
        triangles = [[tri_vertices[0], tri_vertices[1], tri_vertices[2]],
                     [tri_vertices[0], tri_vertices[1], tri_vertices[3]],
                     [tri_vertices[0], tri_vertices[2], tri_vertices[3]],
                     [tri_vertices[1], tri_vertices[2], tri_vertices[3]]
        ]
        # print(f"Triangles in Tetrahedron {tetrahedron}: {triangles}")

        for triangle in triangles:
            triangle_sorted = sorted(triangle)
            total_triangles.append(triangle)
            for index, tri in enumerate(topo_triangles):
                tri_sorted = sorted(tri)
                if triangle_sorted == tri_sorted:
                    surface_triangles.append(index)
                    vertices.append(root.topo.triangles[index].tolist())

    #         tetrahedra = root.topo.getTetrahedraAroundTriangle(triangle)
    #         print(f"Tetrahedra around Triangle {triangle}: {tetrahedra}")
    #         if len(tetrahedra) == 1:
    #             surface_triangles.append(triangle)
    #             vertices.append(root.topo.triangles[triangle].tolist())
    
    # print(f"Total Triangles {len(total_triangles)}: {total_triangles}")
    print(f"Surface Triangles {len(surface_triangles)}: {surface_triangles}")

    print(vertices)

if __name__ == '__main__':
    main()
