import numpy as np
import triangle as tr
import scipy.io as sio
import meshio

def load_mat_file(filename):
    mat_data = sio.loadmat(filename)
    polygons = [p.tolist() for p in mat_data['regions'][0]]
    return polygons

def generate_mesh(polygons, max_area=0.01):
    segments = []
    points = []
    
    for polygon in polygons:
        start_idx = len(points)
        points.extend(polygon)
        
        for i in range(len(polygon)):
            segments.append([start_idx + i, start_idx + (i + 1) % len(polygon)])
    
    
    poly_dict = {
        'vertices': np.array(points),
        'segments': np.array(segments),
    }
    t = tr.triangulate(poly_dict, f'pq30a{max_area}')
    
    return t

def generate_stl(tri, filename="output.stl"):
    cells = [("triangle", tri['triangles'])]
    mesh = meshio.Mesh(points=tri['vertices'], cells=cells)
    meshio.write(filename, mesh)
    return filename

polygons = load_mat_file("TwoRegionsNonconvexCase4.mat")
tri_mesh = generate_mesh(polygons, max_area=2.0)
stl_file = generate_stl(tri_mesh, "output.stl")
