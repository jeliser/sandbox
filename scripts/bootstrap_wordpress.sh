#!/bin/bash

./bootstrap_webserver.sh

# Download the latest version of wordpress
if [ ! -f .download_wordpress ]; then
  wget http://wordpress.org/latest.tar.gz
  # Enter the wordpress username
  echo -n "What direct should wordpress be installed to: "
  read output_dir
  tar -xzvf latest.tar.gz -C ${output_dir}
  rm latest.tar.gz
fi
touch .download_wordpress

# Determine if the database setup step can be skipped
read -p "Would you like to skip database setup? [Y/n] " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
  exit 1
fi

# Read MYSQL root password
echo -n "Enter MYSQL root password: "
read -s tmpPass
echo
echo -n "Enter password again: "
read -s mysqlPasswd
echo

if [ $tmpPass != $mysqlPasswd ]; then
  echo "Passwords do not match."
  exit 1
fi

# Enter the wordpress username
echo -n "Enter the wordpress username for this instance: "
read wordpressUser

# Enter the wordpress database name
echo -n "Enter the wordpress database name for this instance: "
read wordpressDbName

# Read MYSQL wordpress password
echo -n "Enter mysql password for new username '${wordpressUser}': "
read -s tmpPass
echo
echo -n "Enter password again: "
read -s wpPasswd
echo

if [ $tmpPass != $wpPasswd ]; then
  echo "Passwords do not match."
  exit 1
fi


# Consider adding a bunch of error checking before overwriting settings
mysql -u root --password=${mysqlPasswd} -e "CREATE DATABASE ${wordpressDbName};"
mysql -u root --password=${mysqlPasswd} -e "CREATE USER ${wordpressUser}@localhost;"
mysql -u root --password=${mysqlPasswd} -e "SET PASSWORD FOR ${wordpressUser}@localhost= PASSWORD(\"${wpPasswd}\");"
mysql -u root --password=${mysqlPasswd} -e "GRANT ALL PRIVILEGES ON ${wordpressDbName}.* TO ${wordpressUser}@localhost IDENTIFIED BY '${wpPasswd}';"
mysql -u root --password=${mysqlPasswd} -e "FLUSH PRIVILEGES;"

if [ -f .install_wordpress ]; then
  echo "Wordpress already installed.  Delete .install_wordpress to reinstall"
  exit 1
fi

