
# kant_dao

ROS 2 package that contains all Python code about PDDL DAO. There are two DAO families implemented:
  - `MONGO`: this is a DAO family that uses MongoDB to storage the PDDL knowledge. Besides, the Mongoengine Python library is used to access MongoDB.
  - `ROS2`: this is a DAO family that uses a ROS 2 node to storage the PDDL knowledge. ROS 2 services are used.

## Mongoengine Installation
```shell
sudo pip3 install mongoengine
sudo pip3 install dnspython
```

## Mongo Installation
```shell
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
sudo apt-get install gnupg
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
sudo apt-get update
sudo apt-get install -y mongodb-org
sudo systemctl start mongod
```

## Mongo Compass (Optional)
```shell
wget https://downloads.mongodb.com/compass/mongodb-compass_1.26.1_amd64.deb
sudo dpkg -i mongodb-compass_1.26.1_amd64.deb
mongodb-compass
```

## Example
```shell
sudo service mongod start
ros2 run kant_dao example_node
```
