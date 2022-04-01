
# kant_dao

## Mongoengine Installation
```shell
sudo pip3 install mongoengine
sudo pip3 install dnspython
```

## Mongocxx Installation
```shell
sudo chmod +x install_mongocxx.sh
./install_mongocxx.sh
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
https://docs.mongodb.com/compass/master/install/


## Example
Shell 1
```shell
sudo service mongod start
ros2 run kant_knowledge_base knowledge_base_node.py #(Optional, for node version)
```

Shell 2
```shell
ros2 run kant_dao example_node.py
```
