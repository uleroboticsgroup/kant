
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