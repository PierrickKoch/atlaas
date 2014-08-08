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

transformation = [
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1,
]

def msg_to_cloud_struct(msg):
    data = base64.b64decode( msg['points'] )
    return [ list( struct.unpack('fff', data[i:i+12]) ) + [0.0]
             for i in range(0, len(data) - 12, 12) ]

def msg_to_cloud_numpy(msg):
    data = base64.b64decode( msg['points'] )
    arr  = np.frombuffer(data, dtype=np.float32)
    arr  = arr.reshape(arr.size/3, 3)
    return [list(p)+[0.0] for p in arr]

def main():
    test = atlaas.PyAtlaas()
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
    with pymorse.Morse() as morse:
        while 1:
            msg   = morse.robot.camera.get()
            cfg   = morse.robot.camera.get_configurations()
            cloud = msg_to_cloud_numpy(msg)
            tr = cfg.result()['configurations']['object_to_robot']
            test.merge(cloud, transformation)
            test.save_currents()
            import pdb; pdb.set_trace()

if __name__ == "__main__":
    main()
