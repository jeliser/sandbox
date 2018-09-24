
# Install ansible

## Ubuntu-16.04 / Ubuntu 18.04

Install `ansible`

``` bash
 > sudo apt-get install software-properties-common
 > sudo apt-add-repository ppa:ansible/ansible
 > sudo apt-get update
 > sudo apt-get install ansible python
```

Install `OpenSSH`

```bash
 > sudo apt-get install openssh-server
 > sudo systemctl status ssh.service
```

# Setup SSH keys

``` bash
# If you don't already have an SSH key
> ssh-keygen
# Install your SSH key onto the target (in this case it's your local machine)
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

