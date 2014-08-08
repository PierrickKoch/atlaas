import struct
import base64
import logging

import atlaas
import pymorse
import numpy as np

logger = logging.getLogger("pymorse")
console = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.setLevel(logging.DEBUG)
logger.addHandler(console)

def msg_to_cloud_struct(msg):
    data = base64.b64decode( msg['points'] )
    return [ list( struct.unpack('fff', data[i:i+12]) ) + [0.0]
             for i in range(0, len(data) - 12, 12) ]

def msg_to_cloud_numpy(msg):
    data = base64.b64decode( msg['points'] )
    arr  = np.frombuffer(data, dtype=np.float32)
    return arr.reshape(arr.size/3, 3)
    # return [list(p)+[0.0] for p in arr]

def get_mat(cfg):
    otr = cfg.result()['configurations']['object_to_robot']
    rotation = otr['rotation']
    translation = otr['translation']
    M = np.identity(4)
    M[:3, :3] = rotation
    M[:3, 3] = translation[:3]
    return M.flatten()

def main():
    test = atlaas.PyAtlaas()
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
    with pymorse.Morse() as morse:
        while morse.is_up():
            pose  = morse.robot.pose.get()
            msg   = morse.robot.camera.get()
            cfg   = morse.robot.camera.get_configurations()
            cloud = msg_to_cloud_numpy( msg )
            tr    = get_mat( cfg )
            test.merge(cloud, tr)
            test.save_currents()
            #import pdb; pdb.set_trace()

if __name__ == "__main__":
    main()
