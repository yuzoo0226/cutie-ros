#!/bin/bash

SESSION="cutie_mask_generator_session"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <argument>"
    exit 1
fi

ARG1="$1"

# セッション作成（すでに存在する場合はスキップ）
tmux has-session -t $SESSION 2>/dev/null
if [ $? != 0 ]; then
    tmux new-session -d -s $SESSION

    tmux split-window -h -t $SESSION
    tmux select-pane -L
    tmux split-window -v -t $SESSION
    # tmux split-window -h -t $SESSION
    # tmux select-pane -L
    tmux select-pane -U

    # ペイン1： bring_up
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'env-hsrc' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    # tmux send-keys -t $SESSION 'source ~/usr/tamhome_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION 'clear' C-m
    tmux send-keys -t $SESSION 'roscore'

    # ペイン2： rosbag play
    tmux select-pane -D
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'env-hsrc' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/usr/tamhome_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION C-l
    tmux send-keys -t $SESSION "rosbag play -l -r 0.5 ${ARG1} -s 0"

    tmux select-pane -R
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'env-hsrc' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/usr/tamhome_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/usr/tamhome_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION C-l
    tmux send-keys -t $SESSION "rosrun cutie_ros auto_mask_generator.py"

fi

# セッションにアタッチ
tmux attach-session -t $SESSION