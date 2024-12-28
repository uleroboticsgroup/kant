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

#include "kant_knowledge_base/parser/dto_msg_parser.hpp"

using namespace kant::knowledge_base::parser;

kant_msgs::msg::PddlType DtoMsgParser::type_dto_to_msg(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  kant_msgs::msg::PddlType msg;
  msg.name = pddl_type_dto->get_name();
  return msg;
}

kant_msgs::msg::PddlObject DtoMsgParser::object_dto_to_msg(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  kant_msgs::msg::PddlObject msg;
  msg.type = this->type_dto_to_msg(pddl_object_dto->get_type());
  msg.name = pddl_object_dto->get_name();
  return msg;
}

kant_msgs::msg::PddlPredicate DtoMsgParser::predicate_dto_to_msg(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  kant_msgs::msg::PddlPredicate msg;

  msg.name = pddl_predicate_dto->get_name();

  for (const auto &pddl_type_dto : pddl_predicate_dto->get_types()) {
    msg.types.push_back(this->type_dto_to_msg(pddl_type_dto));
  }

  return msg;
}

kant_msgs::msg::PddlProposition DtoMsgParser::proposition_dto_to_msg(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  kant_msgs::msg::PddlProposition msg;

  msg.predicate =
      this->predicate_dto_to_msg(pddl_proposition_dto->get_predicate());

  msg.is_goal = pddl_proposition_dto->get_is_goal();

  for (const auto &pddl_object_dto : pddl_proposition_dto->get_objects()) {
    msg.objects.push_back(this->object_dto_to_msg(pddl_object_dto));
  }

  return msg;
}

kant_msgs::msg::PddlConditionEffect DtoMsgParser::condition_effect_dto_to_msg(
    std::shared_ptr<kant::dto::PddlConditionEffectDto>
        pddl_condition_effect_dto) {

  kant_msgs::msg::PddlConditionEffect msg;

  msg.predicate =
      this->predicate_dto_to_msg(pddl_condition_effect_dto->get_predicate());

  for (const auto &pddl_object_dto : pddl_condition_effect_dto->get_objects()) {
    msg.objects.push_back(this->object_dto_to_msg(pddl_object_dto));
  }

  msg.time = pddl_condition_effect_dto->get_time();
  msg.is_negative = pddl_condition_effect_dto->get_is_negative();

  return msg;
}

kant_msgs::msg::PddlAction DtoMsgParser::action_dto_to_msg(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  kant_msgs::msg::PddlAction msg;

  msg.name = pddl_action_dto->get_name();
  msg.duration = pddl_action_dto->get_duration();
  msg.durative = pddl_action_dto->get_durative();

  for (const auto &pddl_object_dto : pddl_action_dto->get_parameters()) {
    msg.parameters.push_back(this->object_dto_to_msg(pddl_object_dto));
  }

  for (const auto &pddl_condition_dto : pddl_action_dto->get_conditions()) {
    msg.coditions.push_back(
        this->condition_effect_dto_to_msg(pddl_condition_dto));
  }

  for (const auto &pddl_effect_dto : pddl_action_dto->get_effects()) {
    msg.effects.push_back(this->condition_effect_dto_to_msg(pddl_effect_dto));
  }

  return msg;
}