#! /bin/sh
[ -e caylus50.blend ] || wget http://homepages.laas.fr/pkoch/pub/caylus50.blend
[ -e mana ] && mv mana mana.`date +%s`
[ -e momo ] && mv momo momo.`date +%s`
mkdir mana
mkdir momo
export MORSE_SILENT_PYTHON_CHECK=1
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
    send-keys "cd mana; while true; do python ~/work/atlaas/python/tile_to_region.py; sleep 1; done" C-m \; \
    split-window \; \
    send-keys "cd momo; while true; do python ~/work/atlaas/python/tile_to_region.py; sleep 1; done" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py mana" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py momo" C-m \; \
    new-window \; \
    send-keys "cd mana" C-m \; \
    send-keys "export PYPERO_LIST=http://localhost:8043" C-m \; \
    send-keys "python ~/work/atlaas/tools/pypero.py 123.4 567.8 -98.7 -65.4" C-m \; \
    split-window \; \
    send-keys "cd momo" C-m \; \
    send-keys "export PYPERO_LIST=http://localhost:8042" C-m \; \
    send-keys "python ~/work/atlaas/tools/pypero.py 123.4 567.8 -98.7 -65.4" C-m \; \
