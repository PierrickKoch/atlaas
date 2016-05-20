import os
poses = []
patern = 'cloud.%05i.pos'
while os.path.isfile(patern%len(poses)):
    with open(patern%len(poses)) as f:
        lines = f.readlines()
    parser = lambda label: map(float, [line for line in lines \
        if line.startswith(label)][0].strip().split()[2:])
    yaw,pitch,roll,x,y,z = main_to_origin = parser('mainToOrigin = ')
    poses.append([round(x, 3), round(y, 3)])

with open('poses.json', 'w') as f:
    f.write(str(poses))
