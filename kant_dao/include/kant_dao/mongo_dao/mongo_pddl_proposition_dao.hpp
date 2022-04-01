
#ifndef KANT_MONGO_PDDL_PROPOSITION_DAO_HPP
#define KANT_MONGO_PDDL_PROPOSITION_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/mongo_dao/mongo_dao.hpp"

#include "kant_dao/dao_interface/pddl_proposition_dao.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_object_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_predicate_dao.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoPddlObjectDao;
class MongoPddlPredicateDao;

class MongoPddlPropositionDao
    : public kant::dao::dao_interface::PddlPropositionDao,
      public MongoDao {

protected:
  MongoPddlObjectDao *mongo_pddl_object_dao;
  MongoPddlPredicateDao *mongo_pddl_predicate_dao;

  bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto);
  std::shared_ptr<kant::dto::PddlPropositionDto>
  mongo_to_dto(bsoncxx::document::view doc_value);
  bool validate_dto(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

public:
  MongoPddlPropositionDao(std::string mongo_uri, bool init_instance);
  MongoPddlPropositionDao(MongoPddlObjectDao *mongo_pddl_object_dao);
  MongoPddlPropositionDao(MongoPddlPredicateDao *mongo_pddl_predicate_dao);
  ~MongoPddlPropositionDao();

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_by_predicate(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_no_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> get_all();

  bool exist_in_mongo(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

  bool
  save(std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool
  update(std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool propagate_saving(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool save_update(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

  bool delete_one(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool delete_all();
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
