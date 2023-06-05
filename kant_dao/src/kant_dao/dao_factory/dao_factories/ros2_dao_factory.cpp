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

#include "kant_dao/dao_factory/dao_factories/ros2_dao_factory.hpp"

using namespace kant::dao::dao_factory::dao_factories;

Ros2DaoFactory::Ros2DaoFactory() : Ros2DaoFactory(nullptr) {}

Ros2DaoFactory::Ros2DaoFactory(simple_node::Node *node) {
  this->set_node(node);
}

Ros2DaoFactory::~Ros2DaoFactory() { delete this->node; }

void Ros2DaoFactory::set_node(simple_node::Node *node) { this->node = node; }

kant::dao::ros2_dao::Ros2PddlTypeDao *Ros2DaoFactory::create_pddl_type_dao() {
  return new kant::dao::ros2_dao::Ros2PddlTypeDao(this->node);
}

kant::dao::ros2_dao::Ros2PddlObjectDao *
Ros2DaoFactory::create_pddl_object_dao() {
  return new kant::dao::ros2_dao::Ros2PddlObjectDao(this->node);
}

kant::dao::ros2_dao::Ros2PddlPredicateDao *
Ros2DaoFactory::create_pddl_predicate_dao() {
  return new kant::dao::ros2_dao::Ros2PddlPredicateDao(this->node);
}

kant::dao::ros2_dao::Ros2PddlActionDao *
Ros2DaoFactory::create_pddl_action_dao() {
  return new kant::dao::ros2_dao::Ros2PddlActionDao(this->node);
}

kant::dao::ros2_dao::Ros2PddlPropositionDao *
Ros2DaoFactory::create_pddl_proposition_dao() {
  return new kant::dao::ros2_dao::Ros2PddlPropositionDao(this->node);
}