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
