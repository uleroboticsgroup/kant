

#include <cstdint>
#include <string>
#include <vector>

#include "kant_dto/pddl_type_dto.hpp"

#include "kant_dao/mongo_dao/mongo_pddl_proposition_dao.hpp"

using namespace kant::dao::mongo_dao;

MongoPddlPropositionDao::MongoPddlPropositionDao(std::string mongo_uri,
                                                 bool init_instance)
    : MongoDao(mongo_uri, "pddl_proposition", init_instance) {

  this->mongo_pddl_object_dao = new MongoPddlObjectDao(this);
  this->mongo_pddl_predicate_dao = new MongoPddlPredicateDao(this);
}

MongoPddlPropositionDao::MongoPddlPropositionDao(
    MongoPddlObjectDao *mongo_pddl_object_dao)
    : MongoDao(mongo_pddl_object_dao->get_uri(), "pddl_proposition", false) {

  // upstream propagation
  this->mongo_pddl_object_dao = mongo_pddl_object_dao;
  this->mongo_pddl_predicate_dao = new MongoPddlPredicateDao(this);
}

MongoPddlPropositionDao::MongoPddlPropositionDao(
    MongoPddlPredicateDao *mongo_pddl_predicate_dao)
    : MongoDao(mongo_pddl_predicate_dao->get_uri(), "pddl_proposition", false) {

  // upstream propagation
  this->mongo_pddl_object_dao = new MongoPddlObjectDao(this);
  this->mongo_pddl_predicate_dao = mongo_pddl_predicate_dao;
}

MongoPddlPropositionDao::~MongoPddlPropositionDao() {
  delete this->mongo_pddl_object_dao;
  delete this->mongo_pddl_predicate_dao;
}

bsoncxx::document::value
MongoPddlPropositionDao::dto_to_mongo(kant::dto::Dto *dto) {

  kant::dto::PddlPropositionDto *pddl_proposition_dto =
      static_cast<kant::dto::PddlPropositionDto *>(dto);

  bsoncxx::builder::basic::document basic_builder{};
  basic_builder.append(bsoncxx::builder::basic::kvp(
      "predicate", pddl_proposition_dto->get_predicate()->get_name()));

  bsoncxx::builder::basic::array array_builder{};

  for (const auto &pddl_object_dto : pddl_proposition_dto->get_objects()) {
    array_builder.append(pddl_object_dto->get_name());
  }

  basic_builder.append(bsoncxx::builder::basic::kvp("objects", array_builder));

  basic_builder.append(bsoncxx::builder::basic::kvp(
      "is_goal", pddl_proposition_dto->get_is_goal()));

  return basic_builder.extract();
}

std::shared_ptr<kant::dto::PddlPropositionDto>
MongoPddlPropositionDao::mongo_to_dto(bsoncxx::document::view doc_value) {

  // PREDICATE
  std::string predicate_name =
      std::string(doc_value["predicate"].get_string().value);

  auto pddl_predicate_dto_list =
      this->mongo_pddl_predicate_dao->get(predicate_name);

  // OBJECTS
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;
  std::string object_name;

  for (const auto &mongo_pddl_object : doc_value["objects"].get_array().value) {

    object_name = std::string(mongo_pddl_object.get_string().value);

    auto pddl_object_dto = this->mongo_pddl_object_dao->get(object_name);

    pddl_object_dto_list.push_back(pddl_object_dto);
  }

  // PROPOSITIONS
  std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto =
      std::make_shared<kant::dto::PddlPropositionDto>(
          kant::dto::PddlPropositionDto(pddl_predicate_dto_list,
                                        pddl_object_dto_list));

  pddl_proposition_dto->set_is_goal(doc_value["is_goal"].get_bool());

  return pddl_proposition_dto;
}

bool MongoPddlPropositionDao::validate_dto(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  auto pddl_predicate_dto = pddl_proposition_dto->get_predicate();

  if (pddl_proposition_dto->get_objects().size() !=
      pddl_predicate_dto->get_types().size()) {
    return false;
  }

  for (long unsigned int i = 0; i < pddl_proposition_dto->get_objects().size();
       i++) {

    if (!pddl_proposition_dto->get_objects().at(i)->get_type()->equals(
            pddl_predicate_dto->get_types().at(i))) {
      return false;
    }
  }

  return true;
}

bool MongoPddlPropositionDao::exist_in_mongo(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  for (const auto &aux_pddl_proposition_dto : this->get_by_predicate(
           pddl_proposition_dto->get_predicate()->get_name())) {

    if (pddl_proposition_dto->equals(aux_pddl_proposition_dto)) {
      return true;
    }
  }

  return false;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
MongoPddlPropositionDao::get_by_predicate(std::string predicate_name) {

  mongocxx::cursor cursor =
      this->get_collection().find(bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp("predicate", predicate_name)));

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  for (auto doc : cursor) {
    auto pddl_proposition_dto = this->mongo_to_dto(doc);

    if (this->validate_dto(pddl_proposition_dto)) {
      pddl_proposition_dto_list.push_back(pddl_proposition_dto);
    }
  }

  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
MongoPddlPropositionDao::get_goals() {

  mongocxx::cursor cursor =
      this->get_collection().find(bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp("is_goal", true)));

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  for (auto doc : cursor) {
    auto pddl_proposition_dto = this->mongo_to_dto(doc);

    if (this->validate_dto(pddl_proposition_dto)) {
      pddl_proposition_dto_list.push_back(pddl_proposition_dto);
    }
  }

  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
MongoPddlPropositionDao::get_no_goals() {

  mongocxx::cursor cursor =
      this->get_collection().find(bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp("is_goal", false)));

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  for (auto doc : cursor) {
    auto pddl_proposition_dto = this->mongo_to_dto(doc);

    if (this->validate_dto(pddl_proposition_dto)) {
      pddl_proposition_dto_list.push_back(pddl_proposition_dto);
    }
  }
  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
MongoPddlPropositionDao::get_all() {

  mongocxx::cursor cursor = this->get_collection().find({});

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  for (auto doc : cursor) {
    pddl_proposition_dto_list.push_back(this->mongo_to_dto(doc));
  }

  return pddl_proposition_dto_list;
}

bool MongoPddlPropositionDao::save(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->validate_dto(pddl_proposition_dto)) {
    return false;
  }

  if (this->exist_in_mongo(pddl_proposition_dto)) {
    return false;
  }

  bool succ = this->propagate_saving(pddl_proposition_dto);

  if (!succ) {
    return false;
  }

  return this->mongo_insert(pddl_proposition_dto.get());
}

bool MongoPddlPropositionDao::update(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->validate_dto(pddl_proposition_dto)) {
    return false;
  }

  if (!this->exist_in_mongo(pddl_proposition_dto)) {
    return false;
  }

  bool succ = this->propagate_saving(pddl_proposition_dto);

  if (!succ) {
    return false;
  }

  bsoncxx::builder::basic::array array_builder{};
  for (const auto &pddl_object_dto : pddl_proposition_dto->get_objects()) {
    array_builder.append(pddl_object_dto->get_name());
  }

  this->get_collection().update_one(
      // filter
      bsoncxx::builder::basic::make_document(
          bsoncxx::builder::basic::kvp(
              "predicate", pddl_proposition_dto->get_predicate()->get_name()),
          bsoncxx::builder::basic::kvp("objects", array_builder)),
      // new data
      bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(
          "$set", this->dto_to_mongo(pddl_proposition_dto.get()))));

  return true;
}

bool MongoPddlPropositionDao::propagate_saving(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  // PROPAGATING SAVING
  bool succ;

  // PREDICATE
  succ = this->mongo_pddl_predicate_dao->save_update(
      pddl_proposition_dto->get_predicate());

  if (!succ) {
    return false;
  }

  // OBJECTS
  for (const auto &pddl_type_dto : pddl_proposition_dto->get_objects()) {
    succ = this->mongo_pddl_object_dao->save_update(pddl_type_dto);

    if (!succ) {
      return false;
    }
  }

  return true;
}

bool MongoPddlPropositionDao::save_update(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->exist_in_mongo(pddl_proposition_dto)) {
    return this->save(pddl_proposition_dto);
  } else {
    return this->update(pddl_proposition_dto);
  }
}

bool MongoPddlPropositionDao::delete_one(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->exist_in_mongo(pddl_proposition_dto)) {
    return false;
  }

  return this->mongo_delete(pddl_proposition_dto.get());
}

bool MongoPddlPropositionDao::delete_all() {
  this->get_collection().delete_many({});
  return true;
}