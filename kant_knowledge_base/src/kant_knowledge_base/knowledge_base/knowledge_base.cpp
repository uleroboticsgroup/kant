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

#include "kant_knowledge_base/knowledge_base/knowledge_base.hpp"

using namespace kant::knowledge_base::knowledge_base;

//***********************
// PDDL TYPES
//***********************
std::shared_ptr<kant::dto::PddlTypeDto>
KnowledgeBase::get_type(std::string type_name) {
  if (this->types.find(type_name) != this->types.end()) {
    return this->types.at(type_name);
  } else {
    return nullptr;
  }
}

std::vector<std::shared_ptr<kant::dto::PddlTypeDto>>
KnowledgeBase::get_all_types() {
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> type_list;
  for (const auto &pair : this->types) {
    type_list.push_back(pair.second);
  }
  return type_list;
}

bool KnowledgeBase::save_type(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  std::string type_name = pddl_type_dto->get_name();

  if (this->types.find(type_name) == this->types.end()) {
    this->types.insert({type_name, pddl_type_dto});

  } else {
    this->types.at(type_name)->set_name(pddl_type_dto->get_name());
  }

  return true;
}

bool KnowledgeBase::delete_type(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (this->types.find(pddl_type_dto->get_name()) == this->types.end()) {
    return false;
  }

  // PROPAGATE DELETING
  bool o_succ, p_succ, a_succ;

  for (const auto &pddl_object_dto : this->get_all_objects()) {

    if (pddl_object_dto->get_type()->equals(pddl_type_dto)) {
      o_succ = this->delete_object(pddl_object_dto);

      if (!o_succ) {
        return false;
      }
    }
  }

  for (const auto &pddl_predicate_dto : this->get_all_predicates()) {

    for (const auto &pddl_pred_type_dto : pddl_predicate_dto->get_types()) {

      if (pddl_pred_type_dto->equals(pddl_type_dto)) {
        p_succ = this->delete_predicate(pddl_predicate_dto);

        if (!p_succ) {
          return false;
        }

        break;
      }
    }
  }

  for (const auto &pddl_action_dto : this->get_all_actions()) {

    for (const auto &pddl_parameter_dto : pddl_action_dto->get_parameters()) {

      if (pddl_parameter_dto->get_type()->equals(pddl_type_dto)) {
        a_succ = this->delete_action(pddl_action_dto);

        if (!a_succ) {
          return false;
        }

        break;
      }
    }
  }

  this->types.erase(pddl_type_dto->get_name());

  return true;
}

bool KnowledgeBase::delete_all_types() {
  bool succ;

  for (const auto &pddl_type_dto : this->get_all_types()) {
    succ = this->delete_type(pddl_type_dto);

    if (!succ) {
      return false;
    }
  }

  return true;
}

//***********************
// PDDL OBJECTS
//***********************
std::shared_ptr<kant::dto::PddlObjectDto>
KnowledgeBase::get_object(std::string object_name) {
  if (this->objects.find(object_name) != this->objects.end()) {
    return this->objects.at(object_name);
  } else {
    return nullptr;
  }
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
KnowledgeBase::get_all_objects() {
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> object_list;
  for (const auto &pair : this->objects) {
    object_list.push_back(pair.second);
  }
  return object_list;
}

bool KnowledgeBase::save_object(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  // PROPAGATE SAVING
  bool succ = this->save_type(pddl_object_dto->get_type());
  if (!succ) {
    return false;
  }

  // SAVING OBJECT
  std::string object_name = pddl_object_dto->get_name();

  pddl_object_dto->set_type(
      this->get_type(pddl_object_dto->get_type()->get_name()));

  if (this->objects.find(object_name) == this->objects.end()) {
    this->objects.insert({object_name, pddl_object_dto});
  } else {
    this->objects.at(object_name)->set_name(pddl_object_dto->get_name());
    this->objects.at(object_name)->set_type(pddl_object_dto->get_type());
  }

  return true;
}

bool KnowledgeBase::delete_object(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (this->objects.find(pddl_object_dto->get_name()) == this->objects.end()) {
    return false;
  }

  // PROPAGATE DELETING
  bool succ;

  for (const auto &pddl_proposition_dto : this->get_all_propositions()) {

    for (const auto &pddl_prop_object_dto :
         pddl_proposition_dto->get_objects()) {

      if (pddl_prop_object_dto->equals(pddl_object_dto)) {
        succ = this->delete_proposition(pddl_proposition_dto);

        if (!succ) {
          return false;
        }

        break;
      }
    }
  }

  this->objects.erase(pddl_object_dto->get_name());

  return true;
}

bool KnowledgeBase::delete_all_objects() {
  bool succ;

  for (const auto &pddl_object_dto : this->get_all_objects()) {
    succ = this->delete_object(pddl_object_dto);

    if (!succ) {
      return false;
    }
  }

  return true;
}

//***********************
// PDDDL PREDICATES
//***********************
std::shared_ptr<kant::dto::PddlPredicateDto>
KnowledgeBase::get_predicate(std::string predicate_name) {
  if (this->predicates.find(predicate_name) != this->predicates.end()) {
    return this->predicates.at(predicate_name);
  } else {
    return nullptr;
  }
}

std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
KnowledgeBase::get_all_predicates() {
  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>> predicate_list;
  for (const auto &pair : this->predicates) {
    predicate_list.push_back(pair.second);
  }
  return predicate_list;
}

bool KnowledgeBase::save_predicate(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  // PROPAGATE SAVING
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> type_list;

  for (const auto &type : pddl_predicate_dto->get_types()) {
    bool succ = this->save_type(type);

    if (!succ) {
      return false;
    }

    type_list.push_back(this->get_type(type->get_name()));
  }

  // SAVING PREDICATE
  std::string predicate_name = pddl_predicate_dto->get_name();

  pddl_predicate_dto->set_types(type_list);

  if (this->predicates.find(predicate_name) == this->predicates.end()) {
    this->predicates.insert({predicate_name, pddl_predicate_dto});

  } else {
    this->predicates.at(predicate_name)
        ->set_name(pddl_predicate_dto->get_name());
    this->predicates.at(predicate_name)
        ->set_types(pddl_predicate_dto->get_types());
  }

  return true;
}

bool KnowledgeBase::delete_predicate(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (this->predicates.find(pddl_predicate_dto->get_name()) ==
      this->predicates.end()) {
    return false;
  }

  // PROPAGATE DELETING
  bool succ;

  // propositions
  for (const auto &pddl_proposition_dto : this->get_all_propositions()) {
    if (pddl_proposition_dto->get_predicate()->equals(pddl_predicate_dto)) {
      succ = this->delete_proposition(pddl_proposition_dto);

      if (!succ) {
        return false;
      }
    }
  }

  // actions
  for (const auto &pddl_action_dto : this->get_all_actions()) {

    std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
        pddl_condi_effect_dto_list;

    std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
        pddl_condition_dto_list = pddl_action_dto->get_conditions();

    std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
        pddl_effect_dto_list = pddl_action_dto->get_effects();

    std::copy(pddl_condition_dto_list.begin(), pddl_condition_dto_list.end(),
              std::back_inserter(pddl_condi_effect_dto_list));
    std::copy(pddl_effect_dto_list.begin(), pddl_effect_dto_list.end(),
              std::back_inserter(pddl_condi_effect_dto_list));

    for (const auto &pddl_condi_effect_dto : pddl_condi_effect_dto_list) {

      if (pddl_condi_effect_dto->get_predicate()->equals(pddl_predicate_dto)) {
        succ = this->delete_action(pddl_action_dto);

        if (!succ) {
          return false;
        }

        break;
      }
    }
  }

  this->predicates.erase(pddl_predicate_dto->get_name());

  return true;
}

bool KnowledgeBase::delete_all_predicates() {

  this->predicates.clear();

  // PROPAGATE DELETING

  return true;
}

//***********************
// PDDL ACTION
//***********************
std::shared_ptr<kant::dto::PddlActionDto>
KnowledgeBase::get_action(std::string action_name) {
  if (this->actions.find(action_name) != this->actions.end()) {
    return this->actions.at(action_name);
  } else {
    return nullptr;
  }
}

std::vector<std::shared_ptr<kant::dto::PddlActionDto>>
KnowledgeBase::get_all_actions() {
  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> action_list;
  for (const auto &pair : this->actions) {
    action_list.push_back(pair.second);
  }
  return action_list;
}

bool KnowledgeBase::prepare_condition_effect_to_save(
    std::shared_ptr<kant::dto::PddlConditionEffectDto> pddl_condi_effect_dto,
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  // checking condition/effect

  // checking time and durative
  if (!pddl_action_dto->get_durative() &&
      !pddl_condi_effect_dto->get_time().empty()) {
    return false;
  } else if (pddl_action_dto->get_durative() &&
             pddl_condi_effect_dto->get_time().empty()) {
    return false;
  }

  // checking objects
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list =
      pddl_condi_effect_dto->get_objects();
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_type_dto_list =
      pddl_condi_effect_dto->get_predicate()->get_types();

  // checking len
  if (pddl_object_dto_list.size() != pddl_type_dto_list.size()) {
    return false;
  }

  // checking types and parameters
  bool error = true;
  for (long unsigned int i = 0; i < pddl_object_dto_list.size(); i++) {

    if (!pddl_object_dto_list.at(i)->get_type()->equals(
            pddl_type_dto_list.at(i))) {
      return false;
    }

    error = true;

    for (const auto &param : pddl_action_dto->get_parameters()) {
      if (param->equals(pddl_object_dto_list.at(i))) {
        error = false;
        break;
      }
    }

    if (error) {
      return false;
    }
  }

  // condition/effect predicate
  std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto =
      pddl_condi_effect_dto->get_predicate();

  bool p_succ = this->save_predicate(pddl_condi_effect_dto->get_predicate());

  if (!p_succ) {
    return false;
  }

  pddl_condi_effect_dto->set_predicate(
      this->get_predicate(pddl_condi_effect_dto->get_predicate()->get_name()));

  // condition/effect objects
  bool o_succ;
  for (const auto &pddl_object_dto : pddl_object_dto_list) {
    o_succ = this->save_type(pddl_object_dto->get_type());

    if (!o_succ) {
      return false;
    }

    pddl_object_dto->set_type(
        this->get_type(pddl_object_dto->get_type()->get_name()));
  }

  return true;
}

bool KnowledgeBase::save_action(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  bool succ;

  // PROPAGATE SAVING

  // parameters
  for (const auto &pddl_parameter_dto : pddl_action_dto->get_parameters()) {
    succ = this->save_type(pddl_parameter_dto->get_type());

    if (!succ) {
      return false;
    }

    pddl_parameter_dto->set_type(
        this->get_type(pddl_parameter_dto->get_type()->get_name()));
  }

  // conditions and effects
  for (const auto &pddl_condition_dto : pddl_action_dto->get_conditions()) {
    succ = this->prepare_condition_effect_to_save(pddl_condition_dto,
                                                  pddl_action_dto);
    if (!succ) {
      return false;
    }
  }

  for (const auto &pddl_effect_dto : pddl_action_dto->get_effects()) {
    succ = this->prepare_condition_effect_to_save(pddl_effect_dto,
                                                  pddl_action_dto);
    if (!succ) {
      return false;
    }
  }

  // SAVING ACTION
  std::string action_name = pddl_action_dto->get_name();

  if (this->actions.find(action_name) == this->actions.end()) {
    this->actions.insert({action_name, pddl_action_dto});

  } else {
    this->actions.at(action_name)->set_name(pddl_action_dto->get_name());
    this->actions.at(action_name)
        ->set_durative(pddl_action_dto->get_durative());
    this->actions.at(action_name)
        ->set_duration(pddl_action_dto->get_duration());
    this->actions.at(action_name)
        ->set_parameters(pddl_action_dto->get_parameters());
    this->actions.at(action_name)
        ->set_conditions(pddl_action_dto->get_conditions());
    this->actions.at(action_name)->set_effects(pddl_action_dto->get_effects());
  }

  return true;
}

bool KnowledgeBase::delete_action(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  if (this->actions.find(pddl_action_dto->get_name()) == this->actions.end()) {
    return false;
  }

  this->actions.erase(pddl_action_dto->get_name());

  return true;
}

bool KnowledgeBase::delete_all_actions() {
  this->actions.clear();
  return true;
}

//***********************
// PDDL PROPOSITIONS
//***********************
std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
KnowledgeBase::get_propositions(std::string predicate_name) {

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> proposition_list;

  for (const auto &propo : this->goals) {
    if (propo->get_predicate()->get_name() == predicate_name) {
      proposition_list.push_back(propo);
    }
  }

  for (const auto &propo : this->propositions) {
    if (propo->get_predicate()->get_name() == predicate_name) {
      proposition_list.push_back(propo);
    }
  }

  return proposition_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
KnowledgeBase::get_propositions_goals() {
  return this->goals;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
KnowledgeBase::get_propositions_no_goals() {
  return this->propositions;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
KnowledgeBase::get_all_propositions() {

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> proposition_list;

  for (const auto &propo : this->goals) {
    proposition_list.push_back(propo);
  }

  for (const auto &propo : this->propositions) {
    proposition_list.push_back(propo);
  }

  return proposition_list;
}

bool KnowledgeBase::check_proposition(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list =
      pddl_proposition_dto->get_objects();
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_type_dto_list =
      pddl_proposition_dto->get_predicate()->get_types();

  if (pddl_object_dto_list.size() != pddl_type_dto_list.size()) {

    return false;
  }

  for (long unsigned int i = 0; i < pddl_object_dto_list.size(); i++) {
    if (!pddl_object_dto_list.at(i)->get_type()->equals(
            pddl_type_dto_list.at(i))) {
      return false;
    }
  }

  return true;
}

bool KnowledgeBase::save_proposition(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->check_proposition(pddl_proposition_dto)) {
    return false;
  }

  // PROPAGATE SAVING
  std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto =
      pddl_proposition_dto->get_predicate();

  bool succ = this->save_predicate(pddl_predicate_dto);

  if (!succ) {
    return false;
  }

  pddl_proposition_dto->set_predicate(
      this->get_predicate(pddl_predicate_dto->get_name()));

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;

  for (const auto &pddl_object_dto : pddl_proposition_dto->get_objects()) {

    succ = this->save_object(pddl_object_dto);

    if (!succ) {
      return false;
    }

    pddl_object_dto_list.push_back(
        this->get_object(pddl_object_dto->get_name()));
  }

  pddl_proposition_dto->set_objects(pddl_object_dto_list);

  // SAVING PROPOSITION
  pddl_proposition_dto->set_objects(pddl_object_dto_list);

  if (pddl_proposition_dto->get_is_goal()) {
    this->goals.push_back(pddl_proposition_dto);
  } else {
    this->propositions.push_back(pddl_proposition_dto);
  }

  return true;
}

bool KnowledgeBase::delete_proposition(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  long unsigned int i = 0;
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  if (pddl_proposition_dto->get_is_goal()) {
    pddl_proposition_dto_list = this->goals;
  } else {
    pddl_proposition_dto_list = this->propositions;
  }

  for (i = 0; i < pddl_proposition_dto_list.size(); i++) {
    if (pddl_proposition_dto_list.at(i)->equals(pddl_proposition_dto)) {
      break;
    }
  }

  if (i == pddl_proposition_dto_list.size()) {
    return false;
  }

  if (pddl_proposition_dto->get_is_goal()) {
    this->goals.erase(this->goals.begin() + i);
  } else {
    this->propositions.erase(this->propositions.begin() + i);
  }

  return true;
}

bool KnowledgeBase::delete_all_propositions() {
  this->goals.clear();
  this->propositions.clear();
  return true;
}