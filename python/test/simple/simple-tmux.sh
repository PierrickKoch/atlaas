#! /bin/sh
[ -e laas.blend ] || wget http://homepages.laas.fr/pkoch/pub/laas.blend
[ -e mana ] && mv mana mana.`date +%s`
mkdir mana
tmux att -t simple || tmux \
    new-session -s simple \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "morse run -g 800x600 simple-morse.py" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "roscore" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "rosrun rviz rviz -d simple-rviz.rviz" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python simple-ros-repro.py mana" C-m \; \
