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
    t['vertices'] = np.hstack([t['vertices'], np.zeros((t['vertices'].shape[0], 1))])
    
    # Add additional faces
    additional_faces = []
    for edge in poly_dict['segments']:
        v1, v2 = edge
        midpoint = (t['vertices'][v1] + t['vertices'][v2]) / 2
        midpoint[2] = 1  # Set z = 1
        mid_idx = len(t['vertices'])
        t['vertices'] = np.vstack([t['vertices'], midpoint])
        additional_faces.append([v1, mid_idx, v2])
    
    t['triangles'] = np.vstack([t['triangles'], additional_faces])
    
    return t

def generate_stl(tri, filename="output.stl"):
    cells = [("triangle", tri['triangles'])]
    mesh = meshio.Mesh(points=tri['vertices'], cells=cells)
    meshio.write(filename, mesh)
    return filename

polygons = load_mat_file("../../../../../CoveragePathPlanning/TSPCPP/Fast_NN-2Opt_TSPCPP/TwoRegionsNonconvexCase5b.mat")
tri_mesh = generate_mesh(polygons, max_area=4.0)
stl_file = generate_stl(tri_mesh, "TwoRegionsNonconvexCase5b.stl")
