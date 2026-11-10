"""Read the installed shared description, never import a simulator backend."""
import xml.etree.ElementTree as E
from morla_core.resource_locator import resolve_plugin_resource


def model_xml():
    path=resolve_plugin_resource('mmk2_description','mjcf/mmk2.xml')
    root=E.parse(path).getroot()
    root.find('compiler').set('meshdir',str(path.parent.parent/'meshes'))
    return E.tostring(root,encoding='unicode')
