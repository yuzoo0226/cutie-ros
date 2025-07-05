#!/bin/bash

SESSION="cutie_record_session"

# セッション作成（すでに存在する場合はスキップ）
tmux has-session -t $SESSION 2>/dev/null
if [ $? != 0 ]; then
    tmux new-session -d -s $SESSION

    tmux split-window -h -t $SESSION
    tmux select-pane -L
    tmux split-window -v -t $SESSION
    tmux split-window -h -t $SESSION
    tmux select-pane -L
    tmux select-pane -U

    # ペイン1： bring_up
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'sh 0_env.sh' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/ros_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION 'clear' C-m
    tmux send-keys -t $SESSION 'roslaunch tam_hsr_utils bring_up.launch' C-m

    # ペイン1： bring_up
    tmux select-pane -D
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'sh 0_env.sh' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/ros_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION C-l
    tmux send-keys -t $SESSION 'roslaunch navigation_start navigation.launch map_name:=rc25_3330'

    tmux select-pane -R
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'sh 0_env.sh' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/ros_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION 'clear' C-m
    tmux send-keys -t $SESSION 'rviz -d ~/ros_ws/src/5_skills/cutie-ros/io/rviz/hsr_visualization.rviz' C-m


    tmux select-pane -R
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'sh 0_env.sh' C-m
    tmux send-keys -t $SESSION 'source /entrypoint.sh' C-m
    tmux send-keys -t $SESSION 'source ~/ros_ws/1_hsrb_settings.sh' C-m
    tmux send-keys -t $SESSION 'cd ~/ros_ws/' C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION 'clear' C-m
    tmux send-keys -t $SESSION '' C-m
    tmux send-keys -t $SESSION '~/ros_ws/src/5_skills/cutie-ros/io/hsr_record.sh'

fi

# セッションにアタッチ
tmux attach-session -t $SESSION