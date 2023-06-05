// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
