#! /bin/sh
[ -e mana ] && mv mana mana.`date +%s`
mkdir mana
tmux att -t noisy || tmux \
    new-session -s noisy \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "morse run -g 800x600 noisy-morse.py" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "roscore" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "rosrun rviz rviz -d noisy-rviz.rviz" C-m \; \
    new-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python ghost-ros.py" C-m \; \
    split-window \; \
    send-keys "source /opt/ros/indigo/setup.bash" C-m \; \
    send-keys "python noisy-ros.py mana" C-m \; \
