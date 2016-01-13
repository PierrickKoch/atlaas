#! /bin/sh
[ -e caylus50.blend ] || wget http://homepages.laas.fr/pkoch/pub/caylus50.blend
[ -e mana ] && mv mana mana.`date +%s`
[ -e momo ] && mv momo momo.`date +%s`
mkdir mana
mkdir momo
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
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py mana" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python multi-ros.py momo" C-m \; \
