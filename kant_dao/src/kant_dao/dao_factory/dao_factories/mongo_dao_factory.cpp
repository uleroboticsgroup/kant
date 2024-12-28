// Copyright (C) 2023 Miguel Ángel González Santamarta

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

#include <mongocxx/instance.hpp>

#include "kant_dao/dao_factory/dao_factories/mongo_dao_factory.hpp"

using namespace kant::dao::dao_factory::dao_factories;

MongoDaoFactory::MongoDaoFactory() : MongoDaoFactory("") {}

MongoDaoFactory::MongoDaoFactory(std::string mongo_uri) {
  mongocxx::instance instance{};
  this->set_uri(mongo_uri);
}

void MongoDaoFactory::set_uri(std::string mongo_uri) {
  this->mongo_uri = mongo_uri;
}

kant::dao::mongo_dao::MongoPddlTypeDao *
MongoDaoFactory::create_pddl_type_dao() {
  return new kant::dao::mongo_dao::MongoPddlTypeDao(this->mongo_uri, false);
}

kant::dao::mongo_dao::MongoPddlObjectDao *
MongoDaoFactory::create_pddl_object_dao() {
  return new kant::dao::mongo_dao::MongoPddlObjectDao(this->mongo_uri, false);
}

kant::dao::mongo_dao::MongoPddlPredicateDao *
MongoDaoFactory::create_pddl_predicate_dao() {
  return new kant::dao::mongo_dao::MongoPddlPredicateDao(this->mongo_uri,
                                                         false);
}

kant::dao::mongo_dao::MongoPddlActionDao *
MongoDaoFactory::create_pddl_action_dao() {
  return new kant::dao::mongo_dao::MongoPddlActionDao(this->mongo_uri, false);
}

kant::dao::mongo_dao::MongoPddlPropositionDao *
MongoDaoFactory::create_pddl_proposition_dao() {
  return new kant::dao::mongo_dao::MongoPddlPropositionDao(this->mongo_uri,
                                                           false);
}