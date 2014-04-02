#!/bin/bash
#tmux_sandbox.sh
 
SESSIONNAME="CPP_Sandbox"
tmux has-session -t $SESSIONNAME >> /dev/null
 
if [ $? -ne 0 ]
  then
    # new session with name $SESSIONNAME and window 0 named "base"
    tmux new-session -s $SESSIONNAME -n "Default" -d
 
    # new window for source code directory
    tmux new-window -t $SESSIONNAME -n "Source Code"
#    tmux select-window -t $SESSIONNAME:1
    tmux send-keys 'cd /home/jeliser/code/sandbox/cpp' 'C-m'
 
    ## new window for processwire project
    # new window for executeable directory
    tmux new-window -t $SESSIONNAME -n "Executable"
#    tmux select-window -t $SESSIONNAME:2
    tmux send-keys 'cd /home/jeliser/code/sandbox/cpp' 'C-m'
    tmux send-keys 'mkdir ./bin' 'C-m'
    tmux send-keys 'cd ./bin' 'C-m'
    #tmux split-window -t $SESSIONNAME:1 -h
    #tmux select-pane -t $SESSIONNAME:1 -L
    #tmux send-keys 'cd /home/ryann/projects/processwire/build/' 'C-m'
 
    ## switch the default window
    tmux select-window -t $SESSIONNAME:1
fi
 
tmux attach -t $SESSIONNAME



