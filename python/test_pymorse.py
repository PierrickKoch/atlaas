import time
import base64
import logging

import atlaas
import pymorse
import numpy as np

logger = logging.getLogger("pymorse")
console = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.setLevel(logging.INFO)
logger.addHandler(console)

def msg_to_cloud_numpy(msg):
    data = base64.b64decode( msg['points'] )
    buff = np.frombuffer(data, dtype=np.float32)
    return buff.reshape(buff.size/3, 3)

def main():
    test = atlaas.Atlaas()
    test.init(120.0, 120.0, 0.1, 0, 0, 0, 31, True)
    profile = []
    with pymorse.Morse() as morse:
        try:
            while morse.is_up():
                msg = morse.robot.camera.get()
                start = time.time()
                test.merge( msg_to_cloud_numpy( msg ),
                            np.array(msg['sensor_world']) )
                profile.append(time.time() - start)
        except KeyboardInterrupt:
            print("Bye.")
        finally:
            test.save_currents()
            print("merge min/max/avg = %.3f / %.3f / %.3f ms" % \
                ( 1000*min(profile), 1000*max(profile),
                  1000*sum(profile) / len(profile) ) )

if __name__ == "__main__":
    main()
