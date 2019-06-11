python3 -c "" 2>/dev/null 1>/dev/null

if [[ "$?" == 0 ]]; then
  echo "Valid command"
fi

python3 -c "asdasd" 2>/dev/null 1>/dev/null

if [[ "$?" != 0 ]]; then
  echo "Invalid command"
fi

python3 asdasd 2>/dev/null 1>/dev/null

if [[ "$?" != 0 ]]; then
  echo "Invalid command"
fi

