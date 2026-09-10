"""Load an official URDF, preserve its joints, describe the existing LIO frame."""
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import inverse, rpy


def prepare_description(urdf_text, lightning_config, cloud_alias=''):
    root = ET.fromstring(urdf_text)
    links = {x.attrib['name'] for x in root.findall('link')}
    if not {'pelvis', 'mid360_link'}.issubset(links):
        raise ValueError('Official G1 URDF must contain pelvis and mid360_link')
    # Keep kinematics and limits. Meshes and dynamics are unnecessary for TF.
    for link in root.findall('link'):
        for child in list(link):
            if child.tag in ('visual', 'collision', 'inertial'):
                link.remove(child)
    if 'lightning_tracking' in links:
        raise ValueError('URDF already has lightning_tracking; avoid two definitions')
    f = lightning_config['fasterlio']
    til = np.eye(4)
    til[:3, :3] = np.asarray(f['extrinsic_R'], dtype=float).reshape(3, 3)
    til[:3, 3] = np.asarray(f['extrinsic_T'], dtype=float)
    rotation = til[:3, :3]
    if not np.isfinite(til).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
        raise ValueError('Invalid existing LiDAR-to-IMU extrinsic')
    # p_I = T_I_L p_L. Thus URDF joint L->I uses inverse(T_I_L).
    # This describes the algorithm origin; it never rotates incoming measurements.
    tli = inverse(til)
    ET.SubElement(root, 'link', name='lightning_tracking')
    j = ET.SubElement(root, 'joint', name='lightning_tracking_joint', type='fixed')
    ET.SubElement(j, 'parent', link='mid360_link')
    ET.SubElement(j, 'child', link='lightning_tracking')
    ET.SubElement(j, 'origin', xyz=' '.join(map(str, tli[:3, 3])), rpy=' '.join(map(str, rpy(tli))))
    if cloud_alias:
        if cloud_alias in links or cloud_alias in ('map', 'odom', 'lightning_tracking'):
            raise ValueError('Cloud alias must be a new sensor frame; do not alias a world frame')
        ET.SubElement(root, 'link', name=cloud_alias)
        j = ET.SubElement(root, 'joint', name='g1_cloud_alias_joint', type='fixed')
        ET.SubElement(j, 'parent', link='mid360_link')
        ET.SubElement(j, 'child', link=cloud_alias)
    active = [j.attrib['name'] for j in root.findall('joint') if j.attrib['type'] != 'fixed']
    return ET.tostring(root, encoding='unicode'), active
