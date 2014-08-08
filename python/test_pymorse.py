import struct
import base64
import logging

import atlaas
import pymorse
import numpy as np
from transformations import euler_matrix

logger = logging.getLogger("pymorse")
console = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.setLevel(logging.INFO)
logger.addHandler(console)

def msg_to_cloud_numpy(msg):
    data = base64.b64decode( msg['points'] )
    arr  = np.frombuffer(data, dtype=np.float32)
    return arr.reshape(arr.size/3, 3)

def mat_cfg(cfg):
    otr = cfg.result()['configurations']['object_to_robot']
    rotation = otr['rotation']
    translation = otr['translation']
    M = np.identity(4)
    M[:3, :3] = rotation
    M[:3, 3] = translation[:3]
    return M

def mat_pose(pose):
    rotation = [pose[k] for k in ('roll','pitch','yaw')]
    translation = [pose[k] for k in ('x','y','z')]
    M = euler_matrix(*rotation)
    M[:3, 3] = translation[:3]
    return M

def main():
    test = atlaas.Atlaas()
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
    with pymorse.Morse() as morse:
        cfg = morse.robot.camera.get_configurations()
        cam_mat = mat_cfg(cfg)
        while morse.is_up():
            pose = morse.robot.pose.get()
            msg = morse.robot.camera.get()
            cloud = msg_to_cloud_numpy( msg )
            rob_mat = mat_pose(pose)
            tr = rob_mat.dot(cam_mat)
            import pdb; pdb.set_trace()
            test.merge( cloud, tr.flatten() )
            test.save_currents()
            #import pdb; pdb.set_trace()

if __name__ == "__main__":
    main()
