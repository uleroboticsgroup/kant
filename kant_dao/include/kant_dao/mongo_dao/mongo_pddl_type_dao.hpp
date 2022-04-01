
#ifndef KANT_MONGO_PDDL_TYPE_DAO_HPP
#define KANT_MONGO_PDDL_TYPE_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/mongo_dao/mongo_dao.hpp"

#include "kant_dao/dao_interface/pddl_type_dao.hpp"
#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_action_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_object_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_predicate_dao.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoPddlObjectDao;
class MongoPddlPredicateDao;
class MongoPddlActionDao;

class MongoPddlTypeDao : public kant::dao::dao_interface::PddlTypeDao,
                         public MongoDao {

protected:
  MongoPddlObjectDao *mongo_pddl_object_dao;
  MongoPddlPredicateDao *mongo_pddl_predicate_dao;
  MongoPddlActionDao *mongo_pddl_action_dao;

  bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto);
  std::shared_ptr<kant::dto::PddlTypeDto>
  mongo_to_dto(bsoncxx::document::view doc_value);

public:
  MongoPddlTypeDao(std::string mongo_uri, bool init_instance);
  MongoPddlTypeDao(MongoPddlObjectDao *mongo_pddl_object_dao);
  MongoPddlTypeDao(MongoPddlPredicateDao *mongo_pddl_predicate_dao);
  MongoPddlTypeDao(MongoPddlActionDao *mongo_pddl_action_dao);
  ~MongoPddlTypeDao();

  std::shared_ptr<kant::dto::PddlTypeDto> get(std::string type_name);
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> get_all();

  bool exist_in_mongo(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);

  bool save(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);
  bool update(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);
  bool save_update(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);

  bool delete_one(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);
  bool delete_all();
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
