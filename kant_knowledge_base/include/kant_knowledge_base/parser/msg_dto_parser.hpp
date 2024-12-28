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

#ifndef KANT_PARSER_MSG_DTO_HPP
#define KANT_PARSER_MSG_DTO_HPP

#include <memory>

#include "kant_msgs/msg/pddl_action.hpp"
#include "kant_msgs/msg/pddl_condition_effect.hpp"
#include "kant_msgs/msg/pddl_object.hpp"
#include "kant_msgs/msg/pddl_predicate.hpp"
#include "kant_msgs/msg/pddl_proposition.hpp"
#include "kant_msgs/msg/pddl_type.hpp"

#include "kant_dto/pddl_action_dto.hpp"
#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

namespace kant {
namespace knowledge_base {
namespace parser {

class MsgDtoParser {
public:
  std::shared_ptr<kant::dto::PddlTypeDto>
  type_msg_to_dto(kant_msgs::msg::PddlType pddl_type_msg);

  std::shared_ptr<kant::dto::PddlObjectDto>
  object_msg_to_dto(kant_msgs::msg::PddlObject pddl_object_msg);

  std::shared_ptr<kant::dto::PddlPredicateDto>
  predicate_msg_to_dto(kant_msgs::msg::PddlPredicate pddl_predicate_msg);

  std::shared_ptr<kant::dto::PddlPropositionDto>
  proposition_msg_to_dto(kant_msgs::msg::PddlProposition pddl_proposition_msg);

  std::shared_ptr<kant::dto::PddlConditionEffectDto>
  condition_effect_msg_to_dto(
      kant_msgs::msg::PddlConditionEffect pddl_condition_effect_msg);

  std::shared_ptr<kant::dto::PddlActionDto>
  action_msg_to_dto(kant_msgs::msg::PddlAction pddl_action_msg);
};

} // namespace parser
} // namespace knowledge_base
} // namespace kant

#endif
