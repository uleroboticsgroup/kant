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

#ifndef KANT_DAO_FACTORY_HPP
#define KANT_DAO_FACTORY_HPP

#include "kant_dao/dao_interface/pddl_action_dao.hpp"
#include "kant_dao/dao_interface/pddl_object_dao.hpp"
#include "kant_dao/dao_interface/pddl_predicate_dao.hpp"
#include "kant_dao/dao_interface/pddl_proposition_dao.hpp"
#include "kant_dao/dao_interface/pddl_type_dao.hpp"

namespace kant {
namespace dao {
namespace dao_factory {
namespace dao_factories {

class DaoFactory {

public:
  virtual ~DaoFactory(){};

  virtual kant::dao::dao_interface::PddlTypeDao *create_pddl_type_dao() = 0;
  virtual kant::dao::dao_interface::PddlObjectDao *create_pddl_object_dao() = 0;
  virtual kant::dao::dao_interface::PddlPredicateDao *
  create_pddl_predicate_dao() = 0;
  virtual kant::dao::dao_interface::PddlActionDao *create_pddl_action_dao() = 0;
  virtual kant::dao::dao_interface::PddlPropositionDao *
  create_pddl_proposition_dao() = 0;
};

} // namespace dao_factories
} // namespace dao_factory
} // namespace dao
} // namespace kant

#endif