
#ifndef KANT_PARSER_MSG_DTO_HPP
#define KANT_PARSER_MSG_DTO_HPP

#include <memory>

#include "kant_interfaces/msg/pddl_action.hpp"
#include "kant_interfaces/msg/pddl_condition_effect.hpp"
#include "kant_interfaces/msg/pddl_object.hpp"
#include "kant_interfaces/msg/pddl_predicate.hpp"
#include "kant_interfaces/msg/pddl_proposition.hpp"
#include "kant_interfaces/msg/pddl_type.hpp"

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
  type_msg_to_dto(kant_interfaces::msg::PddlType pddl_type_msg);

  std::shared_ptr<kant::dto::PddlObjectDto>
  object_msg_to_dto(kant_interfaces::msg::PddlObject pddl_object_msg);

  std::shared_ptr<kant::dto::PddlPredicateDto>
  predicate_msg_to_dto(kant_interfaces::msg::PddlPredicate pddl_predicate_msg);

  std::shared_ptr<kant::dto::PddlPropositionDto> proposition_msg_to_dto(
      kant_interfaces::msg::PddlProposition pddl_proposition_msg);

  std::shared_ptr<kant::dto::PddlConditionEffectDto>
  condition_effect_msg_to_dto(
      kant_interfaces::msg::PddlConditionEffect pddl_condition_effect_msg);

  std::shared_ptr<kant::dto::PddlActionDto>
  action_msg_to_dto(kant_interfaces::msg::PddlAction pddl_action_msg);
};

} // namespace parser
} // namespace knowledge_base
} // namespace kant

#endif
