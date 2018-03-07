
# Install ansible

## Ubuntu-16.04

``` bash
 > sudo apt-get install software-properties-common
 > sudo apt-add-repository ppa:ansible/ansible
 > sudo apt-get update
 > sudo apt-get install ansible python
```

# Setup SSH keys

``` bash
# If you don't already have an SSH key
> ssh-keygen

> ssh-copy-id jeliser@127.0.0.1
```

# Example usage

``` bash
# Simple case
[jeliser@ubuntu-vm:~/code/sandbox/ansible]  (git:master:b6372b5) 
 > ansible-playbook --ask-sudo-pass jenkins.yml -e "target=127.0.0.1"

# Different user (eg: demo)
[jeliser@ubuntu-vm:~/code/sandbox/ansible]  (git:master:b6372b5) 
 > ansible-playbook --ask-sudo-pass jenkins.yml -e "target=127.0.0.1" -u demo
```

