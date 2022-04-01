
#ifndef KANT_MONGO_PDDL_PREDICATE_DAO_HPP
#define KANT_MONGO_PDDL_PREDICATE_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/mongo_dao/mongo_dao.hpp"

#include "kant_dao/dao_interface/pddl_predicate_dao.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_action_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_proposition_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_type_dao.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoPddlTypeDao;
class MongoPddlPropositionDao;
class MongoPddlActionDao;

class MongoPddlPredicateDao : public kant::dao::dao_interface::PddlPredicateDao,
                              public MongoDao {

protected:
  MongoPddlTypeDao *mongo_pddl_type_dao;
  MongoPddlPropositionDao *mongo_pddl_proposition_dao;
  MongoPddlActionDao *mongo_pddl_action_dao;

  bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto);
  std::shared_ptr<kant::dto::PddlPredicateDto>
  mongo_to_dto(bsoncxx::document::view doc_value);

public:
  MongoPddlPredicateDao(std::string mongo_uri, bool init_instance);
  MongoPddlPredicateDao(MongoPddlTypeDao *mongo_pddl_type_dao);
  MongoPddlPredicateDao(MongoPddlPropositionDao *mongo_pddl_proposition_dao);
  MongoPddlPredicateDao(MongoPddlActionDao *mongo_pddl_action_dao);
  ~MongoPddlPredicateDao();

  std::shared_ptr<kant::dto::PddlPredicateDto> get(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>> get_all();

  bool exist_in_mongo(
      std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);

  bool save(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool update(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool
  save_update(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);

  bool
  delete_one(std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool delete_all();
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
