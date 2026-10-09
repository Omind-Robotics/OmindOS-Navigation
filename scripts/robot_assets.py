"""Explicit local asset catalog: imported URDFs cannot open paths or fetch URLs."""
from pathlib import Path

RM75_FILES = ('base_link.STL', *(f'link_{i}.STL' for i in range(1, 8)))
MEVIUS2_FILES = ('base_link.STL', *(f'{leg}_{part}_link.STL'
                  for leg in ('FR', 'FL', 'BR', 'BL')
                  for part in ('scapula', 'thigh', 'calf')))
MESH_ASSETS = {
    f'/meshes/rm75/{name}': Path('config/robot_models/realman_rm75/meshes')/name
    for name in RM75_FILES
}
MESH_ASSETS.update({
    f'/meshes/mevius2/{name}': Path('config/robot_models/mevius2/models/meshes')/name
    for name in MEVIUS2_FILES
})


def approved_mesh_url(filename):
    quad_prefix = 'package://mevius2/models/meshes/'
    if isinstance(filename, str) and filename.startswith(quad_prefix):
        name = filename[len(quad_prefix):]
        if name in MEVIUS2_FILES:
            return '/meshes/mevius2/' + name
    prefix = 'package://RM75-B/meshes/'
    if isinstance(filename, str) and filename.startswith(prefix):
        name = filename[len(prefix):]
        if name in RM75_FILES:
            return '/meshes/rm75/' + name
    return None

