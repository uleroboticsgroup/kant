
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
  ~ParameterLoader();
  dao_factory::dao_factories::DaoFactory *get_dao_factory();
};

} // namespace dao
} // namespace kant

#endif
