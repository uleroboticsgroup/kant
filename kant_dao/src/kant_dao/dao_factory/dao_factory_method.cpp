
#include "kant_dao/dao_factory/dao_factories/mongo_dao_factory.hpp"
#include "kant_dao/dao_factory/dao_factories/ros2_dao_factory.hpp"

#include "kant_dao/dao_factory/dao_factory_method.hpp"

using namespace kant::dao::dao_factory;

DaoFactoryMethod::DaoFactoryMethod() {}

dao_factories::DaoFactory *
DaoFactoryMethod::create_dao_factory(DaoFamilies family) {
  return this->create_dao_factory(family, nullptr, "");
}

dao_factories::DaoFactory *
DaoFactoryMethod::create_dao_factory(DaoFamilies family,
                                     simple_node::Node *node) {
  return this->create_dao_factory(family, node, "");
}

dao_factories::DaoFactory *
DaoFactoryMethod::create_dao_factory(DaoFamilies family, std::string uri) {
  return this->create_dao_factory(family, nullptr, uri);
}

dao_factories::DaoFactory *
DaoFactoryMethod::create_dao_factory(DaoFamilies family,
                                     simple_node::Node *node, std::string uri) {
  switch (family) {

  case DaoFamilies::ROS2:
    return new dao_factories::Ros2DaoFactory(node);

  case DaoFamilies::MONGO:
    return new dao_factories::MongoDaoFactory(uri);

  default:
    return nullptr;
  }
}
