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

#ifndef KANT_ROS2_DAO_FACTORY_HPP
#define KANT_ROS2_DAO_FACTORY_HPP

#include "simple_node/node.hpp"

#include "kant_dao/ros2_dao/ros2_pddl_action_dao.hpp"
#include "kant_dao/ros2_dao/ros2_pddl_object_dao.hpp"
#include "kant_dao/ros2_dao/ros2_pddl_predicate_dao.hpp"
#include "kant_dao/ros2_dao/ros2_pddl_proposition_dao.hpp"
#include "kant_dao/ros2_dao/ros2_pddl_type_dao.hpp"

#include "kant_dao/dao_factory/dao_factories/dao_factory.hpp"

namespace kant {
namespace dao {
namespace dao_factory {
namespace dao_factories {

class Ros2DaoFactory : public DaoFactory {

private:
public:
  simple_node::Node *node;
  Ros2DaoFactory();
  Ros2DaoFactory(simple_node::Node *node);
  ~Ros2DaoFactory();
  void set_node(simple_node::Node *node);

  kant::dao::ros2_dao::Ros2PddlTypeDao *create_pddl_type_dao();
  kant::dao::ros2_dao::Ros2PddlObjectDao *create_pddl_object_dao();
  kant::dao::ros2_dao::Ros2PddlPredicateDao *create_pddl_predicate_dao();
  kant::dao::ros2_dao::Ros2PddlActionDao *create_pddl_action_dao();
  kant::dao::ros2_dao::Ros2PddlPropositionDao *create_pddl_proposition_dao();
};

} // namespace dao_factories
} // namespace dao_factory
} // namespace dao
} // namespace kant

#endif