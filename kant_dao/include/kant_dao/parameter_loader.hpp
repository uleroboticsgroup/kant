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

#ifndef KANT_PARAMETER_LOADER_HPP
#define KANT_PARAMETER_LOADER_HPP

#include <memory>
#include <string>
#include <vector>

#include "simple_node/node.hpp"

#include "kant_dao/dao_factory/dao_factories/dao_factory.hpp"
#include "kant_dao/dao_factory/dao_factory_method.hpp"
#include "kant_dao/dao_factory/dao_families.hpp"

namespace kant {
namespace dao {

class ParameterLoader {

private:
  dao_factory::dao_factories::DaoFactory *dao_factory;

public:
  ParameterLoader(simple_node::Node *node);
  dao_factory::dao_factories::DaoFactory *get_dao_factory();
};

} // namespace dao
} // namespace kant

#endif
