
# Host machine requirements

It's required to have `docker` installed on the **host machine** in order to run the `ansible` provisioning scripts.

```bash
 > sudo apt install docker -y
```

# Target machine requirements

There must be an SSH connection available on the target machine in order to have the `ansible` provisioning work correctly.
It's also convenient to have the SSH keys copied over to the target machine.  It's saves a bunch of password prompts while the
provisioning is running.

## Install `OpenSSH`

```bash
 > sudo apt-get install openssh-server
 > sudo systemctl status ssh.service
```

## Setup SSH keys

``` bash
# If you don't already have an SSH key
> ssh-keygen
# Install your SSH key onto the target (in this case it's your local machine)
> ssh-copy-id jeliser@127.0.0.1
```

# Example usages

## Simple case

``` bash
[jeliser@ubuntu-vm:~/code/sandbox/ansible]  (git:master:b6372b5) 
 > ./run_me.sh ansible-playbook -e "target=127.0.0.1" -i inventory/inventory.ini --ask-become-pass bootstrap.yml
```

## Different user (eg: demo)

``` bash
[jeliser@ubuntu-vm:~/code/sandbox/ansible]  (git:master:83b1c75) 
 > ./run_me.sh ansible-playbook -e "target=127.0.0.1" -i inventory/inventory.ini --ask-become-pass -u demo bootstrap.yml
```

## Different user with multiple target machines

``` bash
[jeliser@ubuntu-vm:~/code/sandbox/ansible]  (git:master:83b1c75) 
 > ./run_me.sh ansible-playbook -e "target=192.168.1.206,192.168.1.216,192.168.1.217" -u ubuntu -i inventory/inventory.ini --ask-become-pass bootstrap.yml
Using default tag: latest
latest: Pulling from ansible/ansible-runner
Digest: sha256:bd09ef403f2f90f2c6bd133d7533e939058903f69223c5f12557a49e3aed14bb
Status: Image is up to date for ansible/ansible-runner:latest
docker.io/ansible/ansible-runner:latest

PLAY [Bootstrap a new system] ***************************************************************************************************************

TASK [Gathering Facts] **********************************************************************************************************************
ok: [192.168.1.206]
ok: [192.168.1.217]
ok: [192.168.1.216]

PLAY RECAP **********************************************************************************************************************************
192.168.1.206              : ok=1    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
192.168.1.216              : ok=1    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
192.168.1.217              : ok=1    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
```

## Different user with multiple target machines with different become passwords

You will need to create an [Ansible Vault](https://docs.ansible.com/ansible/latest/user_guide/vault.html#vault) in order to hold
the passwords since all the machine will have differing passwords for the **become** user.  This Ansible Vault allows for a single
password to be given to unlock the vault, and then server the passwords over to the relavent machines without exposing any of the
details or password information.

You can find a relevant example of using the Ansible Vault [here](https://www.cyberciti.biz/faq/how-to-set-and-use-sudo-password-for-ansible-vault/).

``` bash
 > ansible-vault create passwd.yml
New Vault password:
Confirm New Vault password:
```

``` bash
[jeliser@jeliser-thinkpad-x1:~/code/github_sandbox/ansible]  (git:master:66e3bdd) [ahead 1]
 > ./run_me.sh ansible-playbook -e "target=192.168.1.206,192.168.1.217,192.168.1.216" -u ubuntu -i inventory/sample_local_network.ini --ask-vault-pass --extra-vars '@passwd.yml' bootstrap.yml
Using default tag: latest
latest: Pulling from ansible/ansible-runner
Digest: sha256:bd09ef403f2f90f2c6bd133d7533e939058903f69223c5f12557a49e3aed14bb
Status: Image is up to date for ansible/ansible-runner:latest
docker.io/ansible/ansible-runner:latest
Vault password: 

PLAY [Bootstrap a new system] ***************************************************************************************************************

TASK [Gathering Facts] **********************************************************************************************************************
ok: [192.168.1.206]
ok: [192.168.1.217]
ok: [192.168.1.216]

TASK [setup : include_tasks] ****************************************************************************************************************
included: /opt/ansible/roles/setup/tasks/install.yml for 192.168.1.206, 192.168.1.217, 192.168.1.216

TASK [setup : Install Bootstrap Dependencies] ***********************************************************************************************
ok: [192.168.1.206] => (item=git)
ok: [192.168.1.206] => (item=tmux)
ok: [192.168.1.217] => (item=git)
ok: [192.168.1.216] => (item=git)
ok: [192.168.1.217] => (item=tmux)
ok: [192.168.1.216] => (item=tmux)

PLAY RECAP **********************************************************************************************************************************
192.168.1.206              : ok=3    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
192.168.1.216              : ok=3    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
192.168.1.217              : ok=3    changed=0    unreachable=0    failed=0    skipped=0    rescued=0    ignored=0
```
