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

#include <cstdint>
#include <string>
#include <vector>

#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_predicate_dao.hpp"

using namespace kant::dao::mongo_dao;

MongoPddlPredicateDao::MongoPddlPredicateDao(std::string mongo_uri,
                                             bool init_instance)
    : MongoDao(mongo_uri, "pddl_predicate", init_instance) {

  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
  this->mongo_pddl_proposition_dao = new MongoPddlPropositionDao(this);
  this->mongo_pddl_action_dao = new MongoPddlActionDao(this);
}

MongoPddlPredicateDao::MongoPddlPredicateDao(
    MongoPddlTypeDao *mongo_pddl_type_dao)
    : MongoDao(mongo_pddl_type_dao->get_uri(), "pddl_predicate", false) {

  // upstream propagation
  this->mongo_pddl_type_dao = mongo_pddl_type_dao;
  this->mongo_pddl_proposition_dao = new MongoPddlPropositionDao(this);
  this->mongo_pddl_action_dao = new MongoPddlActionDao(this);
}

MongoPddlPredicateDao::MongoPddlPredicateDao(
    MongoPddlPropositionDao *mongo_pddl_proposition_dao)
    : MongoDao(mongo_pddl_proposition_dao->get_uri(), "pddl_predicate", false) {

  // downstream propagation
  this->mongo_pddl_proposition_dao = mongo_pddl_proposition_dao;
  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
  this->mongo_pddl_action_dao = nullptr;
}

MongoPddlPredicateDao::MongoPddlPredicateDao(
    MongoPddlActionDao *mongo_pddl_action_dao)
    : MongoDao(mongo_pddl_action_dao->get_uri(), "pddl_predicate", false) {

  // downstream propagation
  this->mongo_pddl_proposition_dao = nullptr;
  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
  this->mongo_pddl_action_dao = mongo_pddl_action_dao;
}

MongoPddlPredicateDao::~MongoPddlPredicateDao() {
  delete this->mongo_pddl_type_dao;
  delete this->mongo_pddl_proposition_dao;
  delete this->mongo_pddl_action_dao;
}

bsoncxx::document::value
MongoPddlPredicateDao::dto_to_mongo(kant::dto::Dto *dto) {

  kant::dto::PddlPredicateDto *pddl_predicate_dto =
      static_cast<kant::dto::PddlPredicateDto *>(dto);

  bsoncxx::builder::basic::document basic_builder{};
  basic_builder.append(
      bsoncxx::builder::basic::kvp("_id", pddl_predicate_dto->get_name()));

  bsoncxx::builder::basic::array array_builder{};

  for (const auto &pddl_type_dto : pddl_predicate_dto->get_types()) {
    array_builder.append(pddl_type_dto->get_name());
  }

  basic_builder.append(bsoncxx::builder::basic::kvp("types", array_builder));

  return basic_builder.extract();
}

std::shared_ptr<kant::dto::PddlPredicateDto>
MongoPddlPredicateDao::mongo_to_dto(bsoncxx::document::view doc_value) {

  std::string predicate_name = std::string(doc_value["_id"].get_string().value);

  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_type_dto_list;

  for (const auto &mongo_pddl_type : doc_value["types"].get_array().value) {

    auto pddl_type_dto = std::make_shared<kant::dto::PddlTypeDto>(
        std::string(mongo_pddl_type.get_string().value));

    pddl_type_dto_list.push_back(pddl_type_dto);
  }

  std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto =
      std::make_shared<kant::dto::PddlPredicateDto>(
          kant::dto::PddlPredicateDto(predicate_name, pddl_type_dto_list));

  return pddl_predicate_dto;
}

bool MongoPddlPredicateDao::exist_in_mongo(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {
  return this->get(pddl_predicate_dto->get_name()) != nullptr;
}

std::shared_ptr<kant::dto::PddlPredicateDto>
MongoPddlPredicateDao::get(std::string predicate_name) {

  bsoncxx::stdx::optional<bsoncxx::document::value> result =
      this->get_collection().find_one(bsoncxx::builder::stream::document{}
                                      << "_id" << predicate_name
                                      << bsoncxx::builder::stream::finalize);

  if (result) {
    auto doc_value = result.value().view();
    return this->mongo_to_dto(doc_value);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
MongoPddlPredicateDao::get_all() {

  mongocxx::cursor cursor = this->get_collection().find({});
  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
      pddl_predicate_dto_list;

  for (auto doc : cursor) {
    pddl_predicate_dto_list.push_back(this->mongo_to_dto(doc));
  }

  return pddl_predicate_dto_list;
}

bool MongoPddlPredicateDao::save(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (this->exist_in_mongo(pddl_predicate_dto)) {
    return false;
  }

  // PROPAGATING SAVING
  bool succ;

  for (const auto &pddl_type_dto : pddl_predicate_dto->get_types()) {
    succ = this->mongo_pddl_type_dao->save_update(pddl_type_dto);

    if (!succ) {
      return false;
    }
  }

  return this->mongo_insert(pddl_predicate_dto.get());
}

bool MongoPddlPredicateDao::update(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (!this->exist_in_mongo(pddl_predicate_dto)) {
    return false;
  }

  // PROPAGATING SAVING
  bool succ;

  for (const auto &pddl_type_dto : pddl_predicate_dto->get_types()) {
    succ = this->mongo_pddl_type_dao->save_update(pddl_type_dto);

    if (!succ) {
      return false;
    }
  }

  this->get_collection().update_one(
      // filter
      bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp("_id", pddl_predicate_dto->get_name())),
      // new data
      bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(
          "$set", this->dto_to_mongo(pddl_predicate_dto.get()))));

  return true;
}

bool MongoPddlPredicateDao::save_update(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (!this->exist_in_mongo(pddl_predicate_dto)) {
    return this->save(pddl_predicate_dto);
  } else {
    return this->update(pddl_predicate_dto);
  }
}

bool MongoPddlPredicateDao::delete_one(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (!this->exist_in_mongo(pddl_predicate_dto)) {
    return false;
  }

  // PROPAGATE DELETING

  // PROPOSITIONS
  bool succ;

  for (const auto &pddl_proposition_dao :
       this->mongo_pddl_proposition_dao->get_all()) {

    if (pddl_predicate_dto->equals(pddl_proposition_dao->get_predicate())) {
      succ = this->mongo_pddl_proposition_dao->delete_one(pddl_proposition_dao);

      if (!succ) {
        return false;
      }
    }
  }

  // ACTIONS
  bool match = false;
  for (const auto &pddl_action_dto : this->mongo_pddl_action_dao->get_all()) {

    // check conditions
    for (const auto &pddl_condition_effect_dto :
         pddl_action_dto->get_conditions()) {

      if (pddl_predicate_dto->equals(
              pddl_condition_effect_dto->get_predicate())) {
        match = true;
        break;
      }
    }

    // check effects
    if (!match) {
      for (const auto &pddl_condition_effect_dto :
           pddl_action_dto->get_effects()) {

        if (pddl_predicate_dto->equals(
                pddl_condition_effect_dto->get_predicate())) {
          match = true;
          break;
        }
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

  return this->mongo_delete(pddl_predicate_dto.get());
}

bool MongoPddlPredicateDao::delete_all() {
  this->get_collection().delete_many({});
  return true;
}