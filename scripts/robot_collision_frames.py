"""Recognize explicitly declared, physically empty, fixed leaf TCP frames."""


def frame_only_links(root, tcp_links):
    links = {node.get('name'): node for node in root.findall('link')}
    joints = root.findall('joint')
    incoming = {j.find('child').get('link'): j for j in joints}
    parents = {j.find('parent').get('link') for j in joints}
    result = set()
    for declaration in root.findall('omindos_frame_only'):
        if set(declaration.attrib) != {'link'} or len(declaration):
            raise ValueError('frame-only declaration must contain only a link name')
        name = declaration.get('link')
        if name in result or name not in links or name not in tcp_links:
            raise ValueError('frame-only link must be a unique, explicitly mapped TCP')
        link, joint = links[name], incoming.get(name)
        if (link.findall('visual') or link.findall('collision') or link.find('inertial') is not None
                or joint is None or joint.get('type') != 'fixed' or name in parents):
            raise ValueError('only an empty fixed leaf TCP may omit physical collision geometry')
        result.add(name)
    return result
