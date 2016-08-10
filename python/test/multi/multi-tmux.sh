#! /bin/sh
[ -e caylus50.blend ] || wget http://homepages.laas.fr/pkoch/pub/caylus50.blend
[ -e mana ] && mv mana mana.`date +%s`
[ -e momo ] && mv momo momo.`date +%s`
mkdir mana
mkdir momo
ln -s $(pwd)/display.html mana/
ln -s $(pwd)/display.html momo/
export MORSE_SILENT_PYTHON_CHECK=1
export PYTHONPATH=$PYTHONPATH:$DEVEL_BASE/lib/python2.7/dist-packages
tmux att -t multi || tmux \
    new-session -s multi \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "morse run -g 800x600 multi-morse.py" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "roscore" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "rosrun rviz rviz -d multi-rviz.rviz" C-m \; \
    new-window \; \
    send-keys "cd mana; python -m SimpleHTTPServer 8042" C-m \; \
    split-window \; \
    send-keys "cd momo; python -m SimpleHTTPServer 8043" C-m \; \
    new-window \; \
    send-keys "cd mana; python ~/work/atlaas/python/tools/tile_to_region.py" C-m \; \
    split-window \; \
    send-keys "cd momo; python ~/work/atlaas/python/tools/tile_to_region.py" C-m \; \
    new-window \; \
    send-keys "ln -s \$(pwd)/mana/atlaas.jpg /tmp" C-m \; \
    send-keys "cd ~/work/threeviz" C-m \; \
    send-keys "open http://localhost:8044" C-m \; \
    send-keys "python -m SimpleHTTPServer 8044" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py mana" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py momo" C-m \; \
    new-window \; \
    send-keys "mkdir minnie; cd minnie" C-m \; \
    send-keys "export MATLAAS_LIST=\"http://localhost:8042;http://localhost:8043\"" C-m \; \
    send-keys "sleep 10" C-m \; \
    send-keys "python ~/work/atlaas/python/tools/matlaas.py 80 -105 236 65" C-m \; \
    send-keys "open region.png" C-m \; \

# goals=[[175, 65, -3], [162, 60, -2.5], [120, 25, -3], [80, 14, -1], [84, 0, -1], [115, -40, -1], [125, -50, -0.8], [140, -72, -1], [160, -105, 0.8], [190, -70, 0.8], [236, -16, 0.5]]
# x=[x for x,_,_ in goals]
# y=[y for _,y,_ in goals]
# min(x), min(y)
# (80, -105)
# max(x), max(y)
# (236, 65)
