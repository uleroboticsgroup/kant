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

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_action_dao.hpp"

using namespace kant::dao::mongo_dao;

MongoPddlActionDao::MongoPddlActionDao(std::string mongo_uri,
                                       bool init_instance)
    : MongoDao(mongo_uri, "pddl_action", init_instance) {

  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
  this->mongo_pddl_predicate_dao = new MongoPddlPredicateDao(this);
}

MongoPddlActionDao::MongoPddlActionDao(MongoPddlTypeDao *mongo_pddl_type_dao)
    : MongoDao(mongo_pddl_type_dao->get_uri(), "pddl_action", false) {

  // downstream propagation
  this->mongo_pddl_predicate_dao = new MongoPddlPredicateDao(this);
  this->mongo_pddl_type_dao = mongo_pddl_type_dao;
}

MongoPddlActionDao::MongoPddlActionDao(
    MongoPddlPredicateDao *mongo_pddl_predicate_dao)
    : MongoDao(mongo_pddl_predicate_dao->get_uri(), "pddl_action", false) {

  // downstream propagation
  this->mongo_pddl_predicate_dao = mongo_pddl_predicate_dao;
  this->mongo_pddl_type_dao = new MongoPddlTypeDao(this);
}

MongoPddlActionDao::~MongoPddlActionDao() {
  delete this->mongo_pddl_type_dao;
  delete this->mongo_pddl_predicate_dao;
}

// ************************
// TO MONGO
// ************************
bsoncxx::document::value MongoPddlActionDao::dto_to_mongo(kant::dto::Dto *dto) {

  kant::dto::PddlActionDto *pddl_action_dto =
      static_cast<kant::dto::PddlActionDto *>(dto);

  std::vector<std::string> predicates;

  bsoncxx::builder::basic::document basic_builder{};

  // PARAMETERS
  bsoncxx::builder::basic::array parameters_array_builder{};

  for (const auto &pddl_object_dto : pddl_action_dto->get_parameters()) {
    parameters_array_builder.append(this->dto_to_mongo(pddl_object_dto));
  }

  // CONDITIONS
  bsoncxx::builder::basic::array conditions_array_builder{};

  for (const auto &pddl_condition_dto : pddl_action_dto->get_conditions()) {

    auto predicate_name = pddl_condition_dto->get_predicate()->get_name();

    if (std::find(predicates.begin(), predicates.end(), predicate_name) ==
        predicates.end()) {

      predicates.push_back(predicate_name);
    }

    conditions_array_builder.append(this->dto_to_mongo(pddl_condition_dto));
  }

  // EFFECTS
  bsoncxx::builder::basic::array effects_array_builder{};

  for (const auto &pddl_condition_dto : pddl_action_dto->get_effects()) {

    auto predicate_name = pddl_condition_dto->get_predicate()->get_name();

    if (std::find(predicates.begin(), predicates.end(), predicate_name) ==
        predicates.end()) {

      predicates.push_back(predicate_name);
    }

    effects_array_builder.append(this->dto_to_mongo(pddl_condition_dto));
  }

  // PREDICATES
  bsoncxx::builder::basic::array predicates_array_builder{};

  // builder
  basic_builder.append(
      bsoncxx::builder::basic::kvp("_id", pddl_action_dto->get_name()));

  basic_builder.append(
      bsoncxx::builder::basic::kvp("_predicates", predicates_array_builder));

  basic_builder.append(
      bsoncxx::builder::basic::kvp("conditions", conditions_array_builder));

  basic_builder.append(bsoncxx::builder::basic::kvp(
      "duration", pddl_action_dto->get_duration()));

  basic_builder.append(bsoncxx::builder::basic::kvp(
      "durative", pddl_action_dto->get_durative()));

  basic_builder.append(
      bsoncxx::builder::basic::kvp("effects", effects_array_builder));

  basic_builder.append(
      bsoncxx::builder::basic::kvp("parameters", parameters_array_builder));

  return basic_builder.extract();
}

bsoncxx::builder::basic::document MongoPddlActionDao::dto_to_mongo(
    std::shared_ptr<kant::dto::PddlConditionEffectDto>
        pddl_condition_effect_dto) {

  bsoncxx::builder::basic::document condition_effect_basic_builder{};

  condition_effect_basic_builder.append(bsoncxx::builder::basic::kvp(
      "time", pddl_condition_effect_dto->get_time()));
  condition_effect_basic_builder.append(bsoncxx::builder::basic::kvp(
      "is_negative", pddl_condition_effect_dto->get_is_negative()));
  condition_effect_basic_builder.append(bsoncxx::builder::basic::kvp(
      "predicate", pddl_condition_effect_dto->get_predicate()->get_name()));

  // PARAMETERS
  bsoncxx::builder::basic::array parameters_array_builder{};

  for (const auto &pddl_object_dto : pddl_condition_effect_dto->get_objects()) {
    parameters_array_builder.append(this->dto_to_mongo(pddl_object_dto));
  }

  condition_effect_basic_builder.append(
      bsoncxx::builder::basic::kvp("parameters", parameters_array_builder));

  return condition_effect_basic_builder;
}

bsoncxx::builder::basic::document MongoPddlActionDao::dto_to_mongo(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  bsoncxx::builder::basic::document parameter_basic_builder{};

  parameter_basic_builder.append(
      bsoncxx::builder::basic::kvp("name", pddl_object_dto->get_name()));

  parameter_basic_builder.append(bsoncxx::builder::basic::kvp(
      "type", pddl_object_dto->get_type()->get_name()));

  return parameter_basic_builder;
}

// ************************
// TO DTO
// ************************
std::shared_ptr<kant::dto::PddlActionDto>
MongoPddlActionDao::mongo_to_dto(bsoncxx::document::view doc_value) {

  std::string action_name = std::string(doc_value["_id"].get_string().value);

  std::shared_ptr<kant::dto::PddlActionDto> ppdl_action_dto =
      std::make_shared<kant::dto::PddlActionDto>(
          kant::dto::PddlActionDto(action_name));

  ppdl_action_dto->set_durative(doc_value["durative"].get_bool());
  ppdl_action_dto->set_duration(doc_value["duration"].get_int32());

  // PARAMETERS
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;

  for (const auto &mongo_pddl_object :
       doc_value["parameters"].get_array().value) {

    pddl_object_dto_list.push_back(
        this->p_mongo_to_dto(mongo_pddl_object.get_document()));
  }

  ppdl_action_dto->set_parameters(pddl_object_dto_list);

  // CONDITIONS
  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
      pddl_condition_dto_list;

  for (const auto &mongo_pddl_condition :
       doc_value["conditions"].get_array().value) {

    pddl_condition_dto_list.push_back(
        this->ce_mongo_to_dto(mongo_pddl_condition.get_document()));
  }

  ppdl_action_dto->set_conditions(pddl_condition_dto_list);

  // EFFECTS
  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
      pddl_effect_dto_list;

  for (const auto &mongo_pddl_effect : doc_value["effects"].get_array().value) {

    pddl_effect_dto_list.push_back(
        this->ce_mongo_to_dto(mongo_pddl_effect.get_document()));
  }

  ppdl_action_dto->set_effects(pddl_effect_dto_list);

  return ppdl_action_dto;
}

std::shared_ptr<kant::dto::PddlConditionEffectDto>
MongoPddlActionDao::ce_mongo_to_dto(bsoncxx::document::view doc_value) {

  std::string predicate_name =
      std::string(doc_value["predicate"].get_string().value);

  auto pddl_predicate_dto = this->mongo_pddl_predicate_dao->get(predicate_name);

  std::shared_ptr<kant::dto::PddlConditionEffectDto> ppdl_condition_effect_dto =
      std::make_shared<kant::dto::PddlConditionEffectDto>(
          kant::dto::PddlConditionEffectDto(pddl_predicate_dto, {}));

  ppdl_condition_effect_dto->set_is_negative(
      doc_value["is_negative"].get_bool());
  ppdl_condition_effect_dto->set_time(
      std::string(doc_value["time"].get_string().value));

  // PARAMETERS
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;

  for (const auto &mongo_pddl_object :
       doc_value["parameters"].get_array().value) {

    pddl_object_dto_list.push_back(
        this->p_mongo_to_dto(mongo_pddl_object.get_document()));
  }

  ppdl_condition_effect_dto->set_objects(pddl_object_dto_list);

  return ppdl_condition_effect_dto;
}

std::shared_ptr<kant::dto::PddlObjectDto>
MongoPddlActionDao::p_mongo_to_dto(bsoncxx::document::view doc_value) {

  std::string type_name = std::string(doc_value["type"].get_string().value);
  std::string object_name = std::string(doc_value["name"].get_string().value);

  auto pddl_type_dto = std::make_shared<kant::dto::PddlTypeDto>(
      kant::dto::PddlTypeDto(type_name));

  auto pddl_object_dto = std::make_shared<kant::dto::PddlObjectDto>(
      kant::dto::PddlObjectDto(pddl_type_dto, object_name));

  return pddl_object_dto;
}

// ************************
// VALIDATE
// ************************
bool MongoPddlActionDao::validate_dto(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  for (const auto &pddl_condition_effect_dto :
       pddl_action_dto->get_conditions()) {

    if (!this->validate_dto(pddl_action_dto, pddl_condition_effect_dto)) {
      return false;
    }
  }

  for (const auto &pddl_condition_effect_dto : pddl_action_dto->get_effects()) {

    if (!this->validate_dto(pddl_action_dto, pddl_condition_effect_dto)) {
      return false;
    }
  }

  return true;
}

bool MongoPddlActionDao::validate_dto(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto,
    std::shared_ptr<kant::dto::PddlConditionEffectDto>
        pddl_condition_effect_dto) {

  // check durative
  if (!pddl_action_dto->get_durative() &&
      !pddl_condition_effect_dto->get_time().empty()) {
    return false;
  } else if (pddl_action_dto->get_durative() &&
             pddl_condition_effect_dto->get_time().empty()) {
    return false;
  }

  auto pddl_predicate_dto = pddl_condition_effect_dto->get_predicate();

  // check sizes
  if (pddl_condition_effect_dto->get_objects().size() !=
      pddl_predicate_dto->get_types().size()) {
    return false;
  }

  bool match = false;

  for (long unsigned int i = 0;
       i < pddl_condition_effect_dto->get_objects().size(); i++) {

    auto pddl_object_dto = pddl_condition_effect_dto->get_objects().at(i);

    // check types
    if (!pddl_object_dto->get_type()->equals(
            pddl_predicate_dto->get_types().at(i))) {
      return false;
    }

    // check parameters
    for (const auto &aux_pddl_object_dto : pddl_action_dto->get_parameters()) {

      if (pddl_object_dto->equals(aux_pddl_object_dto)) {
        match = true;
        break;
      }
    }

    if (!match) {
      return false;
    }
  }

  return true;
}

// ************************
// DAO
// ************************
bool MongoPddlActionDao::exist_in_mongo(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  return this->get(pddl_action_dto->get_name()) != nullptr;
}

std::shared_ptr<kant::dto::PddlActionDto>
MongoPddlActionDao::get(std::string action_name) {

  bsoncxx::stdx::optional<bsoncxx::document::value> result =
      this->get_collection().find_one(bsoncxx::builder::stream::document{}
                                      << "_id" << action_name
                                      << bsoncxx::builder::stream::finalize);

  if (result) {
    auto doc_value = result.value().view();

    auto pddl_action_dto = this->mongo_to_dto(doc_value);

    if (!this->validate_dto(pddl_action_dto)) {
      return nullptr;
    }

    return pddl_action_dto;
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlActionDto>>
MongoPddlActionDao::get_all() {
  mongocxx::cursor cursor = this->get_collection().find({});
  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> pddl_action_dto_list;

  for (auto doc : cursor) {

    auto pddl_action_dto = this->mongo_to_dto(doc);

    if (this->validate_dto(pddl_action_dto)) {
      pddl_action_dto_list.push_back(pddl_action_dto);
    }
  }

  return pddl_action_dto_list;
}

bool MongoPddlActionDao::save(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  if (!this->validate_dto(pddl_action_dto)) {
    return false;
  }

  if (this->exist_in_mongo(pddl_action_dto)) {
    return false;
  }

  bool succ = this->propagate_saving(pddl_action_dto);

  if (!succ) {
    return false;
  }

  return this->mongo_insert(pddl_action_dto.get());
}

bool MongoPddlActionDao::update(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  if (!this->validate_dto(pddl_action_dto)) {
    return false;
  }

  if (!this->exist_in_mongo(pddl_action_dto)) {
    return false;
  }

  bool succ = this->propagate_saving(pddl_action_dto);

  if (!succ) {
    return false;
  }

  bsoncxx::stdx::optional<mongocxx::result::update> result =
      this->get_collection().update_one(
          // filter
          bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("_id", pddl_action_dto->get_name())),
          // new data
          bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(
              "$set", this->dto_to_mongo(pddl_action_dto.get()))));

  return true;
}

bool MongoPddlActionDao::propagate_saving(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  // PROPAGATING SAVING
  bool succ;

  // PREDICATE (CONDITIONS)
  for (const auto &pddl_condition_effect_dto :
       pddl_action_dto->get_conditions()) {
    succ = this->mongo_pddl_predicate_dao->save_update(
        pddl_condition_effect_dto->get_predicate());

    if (!succ) {
      return false;
    }
  }

  // PREDICATE (EFFECTS)
  for (const auto &pddl_condition_effect_dto : pddl_action_dto->get_effects()) {
    succ = this->mongo_pddl_predicate_dao->save_update(
        pddl_condition_effect_dto->get_predicate());

    if (!succ) {
      return false;
    }
  }

  // TYPES
  for (const auto &pddl_object_dto : pddl_action_dto->get_parameters()) {
    succ = this->mongo_pddl_type_dao->save_update(pddl_object_dto->get_type());

    if (!succ) {
      return false;
    }
  }

  return true;
}

bool MongoPddlActionDao::save_update(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  if (!this->exist_in_mongo(pddl_action_dto)) {
    return this->save(pddl_action_dto);
  } else {
    return this->update(pddl_action_dto);
  }
}

bool MongoPddlActionDao::delete_one(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  if (!this->exist_in_mongo(pddl_action_dto)) {
    return false;
  }

  return this->mongo_delete(pddl_action_dto.get());
}

bool MongoPddlActionDao::delete_all() {
  this->get_collection().delete_many({});
  return true;
}
