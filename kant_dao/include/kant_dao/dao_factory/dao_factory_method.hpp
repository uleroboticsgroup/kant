
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