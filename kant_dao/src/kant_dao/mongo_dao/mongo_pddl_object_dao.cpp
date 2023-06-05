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

#include <cstdint>
#include <string>
#include <vector>

#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_object_dao.hpp"

using namespace kant::dao::mongo_dao;

MongoPddlObjectDao::MongoPddlObjectDao(std::string mongo_uri,
                                       bool init_instance)
    : MongoDao(mongo_uri, "pddl_object", init_instance) {

  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
  this->mongo_pddl_proposition_dao = new MongoPddlPropositionDao(this);
}

MongoPddlObjectDao::MongoPddlObjectDao(MongoPddlTypeDao *mongo_pddl_type_dao)
    : MongoDao(mongo_pddl_type_dao->get_uri(), "pddl_object", false) {

  // upstream propagation
  this->mongo_pddl_type_dao = mongo_pddl_type_dao;
  this->mongo_pddl_proposition_dao = new MongoPddlPropositionDao(this);
}

MongoPddlObjectDao::MongoPddlObjectDao(
    MongoPddlPropositionDao *mongo_pddl_proposition_dao)
    : MongoDao(mongo_pddl_proposition_dao->get_uri(), "pddl_object", false) {

  // downstream propagation
  this->mongo_pddl_proposition_dao = mongo_pddl_proposition_dao;
  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
}

MongoPddlObjectDao::~MongoPddlObjectDao() {
  delete this->mongo_pddl_type_dao;
  delete this->mongo_pddl_proposition_dao;
}

bsoncxx::document::value MongoPddlObjectDao::dto_to_mongo(kant::dto::Dto *dto) {

  auto builder = bsoncxx::builder::stream::document{};
  kant::dto::PddlObjectDto *pddl_object_dto =
      static_cast<kant::dto::PddlObjectDto *>(dto);

  bsoncxx::document::value doc_value =
      builder << "_id" << pddl_object_dto->get_name() << "type"
              << pddl_object_dto->get_type()->get_name()
              << bsoncxx::builder::stream::finalize;

  return doc_value;
}

std::shared_ptr<kant::dto::PddlObjectDto>
MongoPddlObjectDao::mongo_to_dto(bsoncxx::document::view doc_value) {

  auto pddl_type_dto = std::make_shared<kant::dto::PddlTypeDto>(
      std::string(doc_value["type"].get_string().value));

  std::string object_name = std::string(doc_value["_id"].get_string().value);

  std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto =
      std::make_shared<kant::dto::PddlObjectDto>(
          kant::dto::PddlObjectDto(pddl_type_dto, object_name));

  return pddl_object_dto;
}

bool MongoPddlObjectDao::exist_in_mongo(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {
  return this->get(pddl_object_dto->get_name()) != nullptr;
}

std::shared_ptr<kant::dto::PddlObjectDto>
MongoPddlObjectDao::get(std::string object_name) {

  bsoncxx::stdx::optional<bsoncxx::document::value> result =
      this->get_collection().find_one(bsoncxx::builder::stream::document{}
                                      << "_id" << object_name
                                      << bsoncxx::builder::stream::finalize);

  if (result) {
    auto doc_value = result.value().view();
    return this->mongo_to_dto(doc_value);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
MongoPddlObjectDao::get_all() {

  mongocxx::cursor cursor = this->get_collection().find({});
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;

  for (auto doc : cursor) {
    pddl_object_dto_list.push_back(this->mongo_to_dto(doc));
  }

  return pddl_object_dto_list;
}

bool MongoPddlObjectDao::save(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (this->exist_in_mongo(pddl_object_dto)) {
    return false;
  }

  // PROPAGATING SAVING
  bool succ =
      this->mongo_pddl_type_dao->save_update(pddl_object_dto->get_type());

  if (!succ) {
    return false;
  }

  return this->mongo_insert(pddl_object_dto.get());
}

bool MongoPddlObjectDao::update(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (!this->exist_in_mongo(pddl_object_dto)) {
    return false;
  }

  // PROPAGATING SAVING
  bool succ =
      this->mongo_pddl_type_dao->save_update(pddl_object_dto->get_type());

  if (!succ) {
    return false;
  }

  bsoncxx::stdx::optional<mongocxx::result::update> result =
      this->get_collection().update_one(
          // filter
          bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("_id", pddl_object_dto->get_name())),
          // new data
          bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(
              "$set", this->dto_to_mongo(pddl_object_dto.get()))));

  return true;
}

bool MongoPddlObjectDao::save_update(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (!this->exist_in_mongo(pddl_object_dto)) {
    return this->save(pddl_object_dto);
  } else {
    return this->update(pddl_object_dto);
  }
}

bool MongoPddlObjectDao::delete_one(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (!this->exist_in_mongo(pddl_object_dto)) {
    return false;
  }

  // PROPAGATE DELETING

  // PROPOSITIONS
  bool succ, match;
  for (const auto &pddl_proposition_dao :
       this->mongo_pddl_proposition_dao->get_all()) {

    for (const auto &aux_pddl_object_dto :
         pddl_proposition_dao->get_objects()) {

      if (pddl_object_dto->equals(aux_pddl_object_dto)) {
        match = true;
        break;
      }
    }

    if (match) {

      succ = this->mongo_pddl_proposition_dao->delete_one(pddl_proposition_dao);

      if (!succ) {
        return false;
      }

      match = false;
    }
  }

  return this->mongo_delete(pddl_object_dto.get());
}

bool MongoPddlObjectDao::delete_all() {
  this->get_collection().delete_many({});
  return true;
}