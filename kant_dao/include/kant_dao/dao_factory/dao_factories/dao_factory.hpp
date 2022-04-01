
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