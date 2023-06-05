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

#ifndef KANT_DAO_FACTORY_METHOD_HPP
#define KANT_DAO_FACTORY_METHOD_HPP

#include <map>
#include <string>

#include "simple_node/node.hpp"

#include "kant_dao/dao_factory/dao_families.hpp"

#include "kant_dao/dao_factory/dao_factories/dao_factory.hpp"

namespace kant {
namespace dao {
namespace dao_factory {

class DaoFactoryMethod {

public:
  DaoFactoryMethod();
  dao_factories::DaoFactory *create_dao_factory(DaoFamilies family);
  dao_factories::DaoFactory *create_dao_factory(DaoFamilies family,
                                                simple_node::Node *node);
  dao_factories::DaoFactory *create_dao_factory(DaoFamilies family,
                                                std::string uri);
  dao_factories::DaoFactory *create_dao_factory(DaoFamilies family,
                                                simple_node::Node *node,
                                                std::string uri);
};

} // namespace dao_factory
} // namespace dao
} // namespace kant

#endif