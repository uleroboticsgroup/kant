
#include "kant_dao/parameter_loader.hpp"

using namespace kant::dao;

ParameterLoader::ParameterLoader(simple_node::Node *node) {

  // param names
  std::string dao_family_param_name = "dao_family";
  std::string mongo_uri_param_name = "mongo_uri";

  // declaring params
  node->declare_parameter<int>(dao_family_param_name,
                               int(dao_factory::DaoFamilies::ROS2));
  node->declare_parameter<std::string>(mongo_uri_param_name,
                                       "mongodb://localhost:27017/kant");

  // getting params
  int dao_family;
  node->get_parameter(dao_family_param_name, dao_family);

  std::string mongo_uri;
  node->get_parameter(mongo_uri_param_name, mongo_uri);

  // creating dao factory
  dao_factory::DaoFactoryMethod dao_factory_method;

  this->dao_factory = dao_factory_method.create_dao_factory(
      dao_factory::DaoFamilies(dao_family), node, mongo_uri);
}

dao_factory::dao_factories::DaoFactory *ParameterLoader::get_dao_factory() {
  return this->dao_factory;
}
