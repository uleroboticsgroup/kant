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

#include <vector>

#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

using namespace kant::knowledge_base::parser;

std::shared_ptr<kant::dto::PddlTypeDto>
MsgDtoParser::type_msg_to_dto(kant_msgs::msg::PddlType pddl_type_msg) {

  std::shared_ptr<kant::dto::PddlTypeDto> dto;

  dto = std::make_shared<kant::dto::PddlTypeDto>(
      kant::dto::PddlTypeDto(pddl_type_msg.name));

  return dto;
}

std::shared_ptr<kant::dto::PddlObjectDto>
MsgDtoParser::object_msg_to_dto(kant_msgs::msg::PddlObject pddl_object_msg) {

  std::shared_ptr<kant::dto::PddlObjectDto> dto;

  dto = std::make_shared<kant::dto::PddlObjectDto>(kant::dto::PddlObjectDto(
      this->type_msg_to_dto(pddl_object_msg.type), pddl_object_msg.name));

  return dto;
}

std::shared_ptr<kant::dto::PddlPredicateDto> MsgDtoParser::predicate_msg_to_dto(
    kant_msgs::msg::PddlPredicate pddl_predicate_msg) {

  std::shared_ptr<kant::dto::PddlPredicateDto> dto;

  dto = std::make_shared<kant::dto::PddlPredicateDto>(
      kant::dto::PddlPredicateDto(pddl_predicate_msg.name));

  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_types_list;

  for (const auto &pddl_type_msg : pddl_predicate_msg.types) {
    pddl_types_list.push_back(this->type_msg_to_dto(pddl_type_msg));
  }

  dto->set_types(pddl_types_list);

  return dto;
}

std::shared_ptr<kant::dto::PddlPropositionDto>
MsgDtoParser::proposition_msg_to_dto(
    kant_msgs::msg::PddlProposition pddl_proposition_msg) {

  std::shared_ptr<kant::dto::PddlPropositionDto> dto;

  dto = std::make_shared<kant::dto::PddlPropositionDto>(
      kant::dto::PddlPropositionDto(
          this->predicate_msg_to_dto(pddl_proposition_msg.predicate), {}));

  dto->set_is_goal(pddl_proposition_msg.is_goal);

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_objects_list;

  for (const auto &pddl_object_msg : pddl_proposition_msg.objects) {
    pddl_objects_list.push_back(this->object_msg_to_dto(pddl_object_msg));
  }

  dto->set_objects(pddl_objects_list);

  return dto;
}

std::shared_ptr<kant::dto::PddlConditionEffectDto>
MsgDtoParser::condition_effect_msg_to_dto(
    kant_msgs::msg::PddlConditionEffect pddl_condition_effect_msg) {

  std::shared_ptr<kant::dto::PddlConditionEffectDto> dto;

  dto = std::make_shared<kant::dto::PddlConditionEffectDto>(
      kant::dto::PddlConditionEffectDto(
          this->predicate_msg_to_dto(pddl_condition_effect_msg.predicate), {}));

  dto->set_is_negative(pddl_condition_effect_msg.is_negative);

  dto->set_time(pddl_condition_effect_msg.time);

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_objects_list;

  for (const auto &pddl_object_msg : pddl_condition_effect_msg.objects) {
    pddl_objects_list.push_back(this->object_msg_to_dto(pddl_object_msg));
  }

  dto->set_objects(pddl_objects_list);

  return dto;
}

std::shared_ptr<kant::dto::PddlActionDto>
MsgDtoParser::action_msg_to_dto(kant_msgs::msg::PddlAction pddl_action_msg) {

  std::shared_ptr<kant::dto::PddlActionDto> dto;

  dto = std::make_shared<kant::dto::PddlActionDto>(
      kant::dto::PddlActionDto(pddl_action_msg.name));

  dto->set_duration(pddl_action_msg.duration);
  dto->set_durative(pddl_action_msg.durative);

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_parameters_list;
  for (const auto &pddl_object_msg : pddl_action_msg.parameters) {
    pddl_parameters_list.push_back(this->object_msg_to_dto(pddl_object_msg));
  }
  dto->set_parameters(pddl_parameters_list);

  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
      pddl_coditions_list;
  for (const auto &pddl_condition_msg : pddl_action_msg.coditions) {
    pddl_coditions_list.push_back(
        this->condition_effect_msg_to_dto(pddl_condition_msg));
  }
  dto->set_conditions(pddl_coditions_list);

  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
      pddl_effect_list;
  for (const auto &pddl_effect_msg : pddl_action_msg.effects) {
    pddl_effect_list.push_back(
        this->condition_effect_msg_to_dto(pddl_effect_msg));
  }
  dto->set_effects(pddl_effect_list);

  return dto;
}
