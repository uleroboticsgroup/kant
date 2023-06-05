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

#include "kant_dao/mongo_dao/mongo_pddl_type_dao.hpp"

using namespace kant::dao::mongo_dao;

MongoPddlTypeDao::MongoPddlTypeDao(std::string mongo_uri, bool init_instance)
    : MongoDao(mongo_uri, "pddl_type", init_instance) {

  this->mongo_pddl_object_dao = new MongoPddlObjectDao(this);
  this->mongo_pddl_predicate_dao = new MongoPddlPredicateDao(this);
  this->mongo_pddl_action_dao = new MongoPddlActionDao(this);
}

MongoPddlTypeDao::MongoPddlTypeDao(MongoPddlObjectDao *mongo_pddl_object_dao)
    : MongoDao(mongo_pddl_object_dao->get_uri(), "pddl_type", false) {

  // downstream propagation
  this->mongo_pddl_object_dao = mongo_pddl_object_dao;
  this->mongo_pddl_predicate_dao = nullptr;
  this->mongo_pddl_action_dao = nullptr;
}

MongoPddlTypeDao::MongoPddlTypeDao(
    MongoPddlPredicateDao *mongo_pddl_predicate_dao)
    : MongoDao(mongo_pddl_predicate_dao->get_uri(), "pddl_type", false) {

  // downstream propagation
  this->mongo_pddl_object_dao = nullptr;
  this->mongo_pddl_predicate_dao = mongo_pddl_predicate_dao;
  this->mongo_pddl_action_dao = nullptr;
}

MongoPddlTypeDao::MongoPddlTypeDao(MongoPddlActionDao *mongo_pddl_action_dao)
    : MongoDao(mongo_pddl_action_dao->get_uri(), "pddl_type", false) {

  // downstream propagation
  this->mongo_pddl_object_dao = nullptr;
  this->mongo_pddl_predicate_dao = nullptr;
  this->mongo_pddl_action_dao = mongo_pddl_action_dao;
}

MongoPddlTypeDao::~MongoPddlTypeDao() {

  delete this->mongo_pddl_object_dao;
  delete this->mongo_pddl_predicate_dao;
  delete this->mongo_pddl_action_dao;
}

bsoncxx::document::value MongoPddlTypeDao::dto_to_mongo(kant::dto::Dto *dto) {

  auto builder = bsoncxx::builder::stream::document{};
  kant::dto::PddlTypeDto *pddl_type_dto =
      static_cast<kant::dto::PddlTypeDto *>(dto);

  bsoncxx::document::value doc_value =
      builder << "_id" << pddl_type_dto->get_name() << "_aux"
              << "AUX" << bsoncxx::builder::stream::finalize;

  return doc_value;
}

std::shared_ptr<kant::dto::PddlTypeDto>
MongoPddlTypeDao::mongo_to_dto(bsoncxx::document::view doc_value) {

  std::string type_name = std::string(doc_value["_id"].get_string().value);

  std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto =
      std::make_shared<kant::dto::PddlTypeDto>(
          kant::dto::PddlTypeDto(type_name));

  return pddl_type_dto;
}

bool MongoPddlTypeDao::exist_in_mongo(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {
  return this->get(pddl_type_dto->get_name()) != nullptr;
}

std::shared_ptr<kant::dto::PddlTypeDto>
MongoPddlTypeDao::get(std::string type_name) {

  bsoncxx::stdx::optional<bsoncxx::document::value> result =
      this->get_collection().find_one(bsoncxx::builder::stream::document{}
                                      << "_id" << type_name
                                      << bsoncxx::builder::stream::finalize);

  if (result) {
    auto doc_value = result.value().view();
    return this->mongo_to_dto(doc_value);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlTypeDto>>
MongoPddlTypeDao::get_all() {

  mongocxx::cursor cursor = this->get_collection().find({});
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_type_dto_list;

  for (auto doc : cursor) {
    pddl_type_dto_list.push_back(this->mongo_to_dto(doc));
  }

  return pddl_type_dto_list;
}

bool MongoPddlTypeDao::save(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (this->exist_in_mongo(pddl_type_dto)) {
    return false;
  }

  return this->mongo_insert(pddl_type_dto.get());
}

bool MongoPddlTypeDao::update(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (!this->exist_in_mongo(pddl_type_dto)) {
    return false;
  }

  this->get_collection().update_one(
      // filter
      bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp("_id", pddl_type_dto->get_name())),
      // new data
      bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(
          "$set", this->dto_to_mongo(pddl_type_dto.get()))));

  return true;
}

bool MongoPddlTypeDao::save_update(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (!this->exist_in_mongo(pddl_type_dto)) {
    return this->save(pddl_type_dto);
  } else {
    return this->update(pddl_type_dto);
  }
}

bool MongoPddlTypeDao::delete_one(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (!this->exist_in_mongo(pddl_type_dto)) {
    return false;
  }

  // PROPAGATE DELETING
  bool succ;

  // OBJECTS
  for (const auto &pddl_object_dto : this->mongo_pddl_object_dao->get_all()) {

    if (pddl_type_dto->equals(pddl_object_dto->get_type())) {
      succ = this->mongo_pddl_object_dao->delete_one(pddl_object_dto);

      if (!succ) {
        return false;
      }
    }
  }

  // PREDICATES
  bool match = false;
  for (const auto &pddl_predicate_dto :
       this->mongo_pddl_predicate_dao->get_all()) {

    for (const auto &aux_pddl_type_dto : pddl_predicate_dto->get_types()) {
      if (pddl_type_dto->equals(aux_pddl_type_dto)) {
        match = true;
        break;
      }
    }

    if (match) {

      succ = this->mongo_pddl_predicate_dao->delete_one(pddl_predicate_dto);

      if (!succ) {
        return false;
      }

      match = false;
    }
  }

  // ACTIONS
  match = false;
  for (const auto &pddl_action_dto : this->mongo_pddl_action_dao->get_all()) {

    for (const auto &pddl_object_dto : pddl_action_dto->get_parameters()) {
      if (pddl_type_dto->equals(pddl_object_dto->get_type())) {
        match = true;
        break;
      }
    }

    if (match) {

      succ = this->mongo_pddl_action_dao->delete_one(pddl_action_dto);

      if (!succ) {
        return false;
      }

      match = false;
    }
  }

  return this->mongo_delete(pddl_type_dto.get());
}

bool MongoPddlTypeDao::delete_all() {

  bool succ;

  for (const auto &pddl_type_dto : this->get_all()) {

    succ = this->delete_one(pddl_type_dto);

    if (!succ) {
      return false;
    }
  }

  return true;
}