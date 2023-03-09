pgrep $1 > /dev/null && echo "$1 is running" || echo "$1 is NOT running"

