"""Explicit local asset catalog: imported URDFs cannot open paths or fetch URLs."""
from pathlib import Path

RM75_FILES = ('base_link.STL', *(f'link_{i}.STL' for i in range(1, 8)))
MESH_ASSETS = {
    f'/meshes/rm75/{name}': Path('config/robot_models/realman_rm75/meshes')/name
    for name in RM75_FILES
}


def approved_mesh_url(filename):
    prefix = 'package://RM75-B/meshes/'
    if isinstance(filename, str) and filename.startswith(prefix):
        name = filename[len(prefix):]
        if name in RM75_FILES:
            return '/meshes/rm75/' + name
    return None
