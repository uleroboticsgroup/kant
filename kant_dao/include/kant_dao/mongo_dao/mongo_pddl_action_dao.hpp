// Copyright (C) 2023 Miguel Ángel González Santamarta

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

#ifndef KANT_MONGO_PDDL_ACTION_DAO_HPP
#define KANT_MONGO_PDDL_ACTION_DAO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/mongo_dao/mongo_dao.hpp"

#include "kant_dao/dao_interface/pddl_action_dao.hpp"
#include "kant_dto/pddl_action_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_predicate_dao.hpp"
#include "kant_dao/mongo_dao/mongo_pddl_type_dao.hpp"

namespace kant {
namespace dao {
namespace mongo_dao {

class MongoPddlTypeDao;
class MongoPddlPredicateDao;

class MongoPddlActionDao : public kant::dao::dao_interface::PddlActionDao,
                           public MongoDao {

protected:
  MongoPddlTypeDao *mongo_pddl_type_dao;
  MongoPddlPredicateDao *mongo_pddl_predicate_dao;

  bsoncxx::document::value dto_to_mongo(kant::dto::Dto *dto);
  bsoncxx::builder::basic::document
  dto_to_mongo(std::shared_ptr<kant::dto::PddlConditionEffectDto>
                   pddl_condition_effect_dto);
  bsoncxx::builder::basic::document
  dto_to_mongo(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);

  std::shared_ptr<kant::dto::PddlActionDto>
  mongo_to_dto(bsoncxx::document::view doc_value);
  std::shared_ptr<kant::dto::PddlConditionEffectDto>
  ce_mongo_to_dto(bsoncxx::document::view doc_value);
  std::shared_ptr<kant::dto::PddlObjectDto>
  p_mongo_to_dto(bsoncxx::document::view doc_value);

  bool validate_dto(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool validate_dto(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto,
                    std::shared_ptr<kant::dto::PddlConditionEffectDto>
                        pddl_condition_effect_dto);

public:
  MongoPddlActionDao(std::string mongo_uri, bool init_instance);
  MongoPddlActionDao(MongoPddlTypeDao *mongo_pddl_type_dao);
  MongoPddlActionDao(MongoPddlPredicateDao *mongo_pddl_predicate_dao);
  ~MongoPddlActionDao();

  std::shared_ptr<kant::dto::PddlActionDto> get(std::string action_name);
  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> get_all();

  bool
  exist_in_mongo(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);

  bool save(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool update(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool
  propagate_saving(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool save_update(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);

  bool delete_one(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool delete_all();
};

} // namespace mongo_dao
} // namespace dao
} // namespace kant

#endif
