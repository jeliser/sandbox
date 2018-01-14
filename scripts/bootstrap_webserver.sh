#!/bin/bash

# Which major distro of linux are we using
REDHAT="redhat"
UBUNTU="ubuntu"
if [ $( which yum 2>/dev/null | wc -l ) -ne 0 ]; then
  distro=${REDHAT}
elif [ $( which apt-get 2>/dev/null | wc -l ) -ne 0 ]; then
  distro=${UBUNTU}
else 
  echo "Unable to determine operating system"
  exit 1
fi

# Check that MYSQL is already installed
if [ $( which mysql 2>/dev/null | wc -l ) -eq 0 ]; then
  echo "Installing MYSQL"
  if [ ${distro} = ${REDHAT} ] ; then
    sudo yum -y install mariadb mariadb-libs mariadb-server mysql
    sudo systemctl enable mariadb.service
    sudo systemctl start mariadb.service
  fi
fi

# Check that APACHE is already installed
if [ $( which httpd 2>/dev/null | wc -l ) -eq 0 ]; then
  echo "Installing Apache"
  if [ ${distro} = ${REDHAT} ] ; then 
    sudo yum -y install httpd
    sudo systemctl enable httpd.service
    sudo systemctl start httpd.service
  fi
fi

# Check that PHP is installed
if [ $( which php 2>/dev/null | wc -l ) -eq 0 ]; then
  echo "Installing PHP and myPhpAdmin"
  if [ ${distro} = ${REDHAT} ] ; then 
    sudo yum -y install php php-mysql php-pdo php-gd phpMyAdmin --skip-broken
  fi
fi

if [ ${distro} = ${UBUNTU} ] ; then
  sudo apt-get -y install lamp-server^
  sudo apt-get -y install phpMyAdmin
  sudo apt-get -y install vsftpd
  sudo apt-get -y install postfix apache2-utils
  sudo php5enmod mcrypt
  sudo a2enmod rewrite
  sudo service apache2 restart
  sudo service vsftpd restart

  echo "Include the following in /etc/apache2/apache2.conf"
  echo "# Include the phpMyAdmin conf file"
  echo "Include /etc/phpmyadmin/apache.conf"

  # Ruby on rails
  # http://www.frontcoded.com/rails-on-amazon-ec2-ubuntu.html
  sudo apt-get -y install ruby-full build-essential
  sudo apt-get -y install rubygems-integration sqlite3 libsqlite3-dev nodejs npm
  sudo gem install rdoc
  sudo gem install rails
  sudo gem install sqlite3-ruby

  #http://www.frontcoded.com/rails-with-apache-ec2-ubuntu.html
  sudo gem install passenger
  sudo apt-get -y install libapache2-mod-passenger libcurl4-openssl-dev
  sudo apt-get -y install apache2-threaded-dev libapr1-dev libaprutil-dev
  echo "You must run: sudo ./bin/passenger-install-apache2-module"

  sudo apt-get -y install openjdk-7-jdk openjdk-7-jre-headless
fi

# http://superuser.com/questions/19318/how-can-i-give-write-access-of-a-folder-to-all-users-in-linux/19333#19333
# Adding group permissions a bunch of the www directories.

# http://stephen-white.blogspot.com/2012/05/how-to-set-up-wordpress-on-amazon-ec2_31.html
# Setup the additional items required for a wordpress site.

echo "Webserver installation complete"




