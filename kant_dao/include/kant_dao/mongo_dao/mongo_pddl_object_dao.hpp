
#ifndef KANT_MONGO_PDDL_OBJECT_DAO_HPP
#define KANT_MONGO_PDDL_OBJECT_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/mongo_dao/mongo_dao.hpp"

#include "kant_dao/dao_interface/pddl_object_dao.hpp"
#include "kant_dto/pddl_object_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_proposition_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_type_dao.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoPddlTypeDao;
class MongoPddlPropositionDao;

class MongoPddlObjectDao : public kant::dao::dao_interface::PddlObjectDao,
                           public MongoDao {

protected:
  MongoPddlTypeDao *mongo_pddl_type_dao;
  MongoPddlPropositionDao *mongo_pddl_proposition_dao;

  bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto);
  std::shared_ptr<kant::dto::PddlObjectDto>
  mongo_to_dto(bsoncxx::document::view doc_value);

public:
  MongoPddlObjectDao(std::string mongo_uri, bool init_instance);
  MongoPddlObjectDao(MongoPddlTypeDao *mongo_pddl_type_dao);
  MongoPddlObjectDao(MongoPddlPropositionDao *mongo_pddl_proposition_dao);
  ~MongoPddlObjectDao();

  std::shared_ptr<kant::dto::PddlObjectDto> get(std::string object_name);
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> get_all();

  bool
  exist_in_mongo(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);

  bool save(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool update(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool save_update(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);

  bool delete_one(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool delete_all();
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
