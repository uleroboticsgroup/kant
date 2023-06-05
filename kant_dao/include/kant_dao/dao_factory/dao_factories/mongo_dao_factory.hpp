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

#ifndef KANT_MONGO_DAO_FACTORY_HPP
#define KANT_MONGO_DAO_FACTORY_HPP

#include <string>

#include "kant_dao/mongo_dao/mongo_pddl_action_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_object_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_predicate_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_proposition_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_type_dao.hpp"

#include "kant_dao/dao_factory/dao_factories/dao_factory.hpp"

namespace kant {
namespace dao {
namespace dao_factory {
namespace dao_factories {

class MongoDaoFactory : public DaoFactory {

private:
public:
  std::string mongo_uri;
  MongoDaoFactory();
  MongoDaoFactory(std::string mongo_uri);
  void set_uri(std::string mongo_uri);

  kant::dao::mongo_dao::MongoPddlTypeDao *create_pddl_type_dao();
  kant::dao::mongo_dao::MongoPddlObjectDao *create_pddl_object_dao();
  kant::dao::mongo_dao::MongoPddlPredicateDao *create_pddl_predicate_dao();
  kant::dao::mongo_dao::MongoPddlActionDao *create_pddl_action_dao();
  kant::dao::mongo_dao::MongoPddlPropositionDao *create_pddl_proposition_dao();
};

} // namespace dao_factories
} // namespace dao_factory
} // namespace dao
} // namespace kant

#endif