
#ifndef KANT_PARSER_DTO_MSG_HPP
#define KANT_PARSER_DTO_MSG_HPP

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

class DtoMsgParser {
public:
  kant_interfaces::msg::PddlType
  type_dto_to_msg(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);

  kant_interfaces::msg::PddlObject
  object_dto_to_msg(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);

  kant_interfaces::msg::PddlPredicate predicate_dto_to_msg(
      std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);

  kant_interfaces::msg::PddlProposition proposition_dto_to_msg(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);

  kant_interfaces::msg::PddlConditionEffect
  condition_effect_dto_to_msg(std::shared_ptr<kant::dto::PddlConditionEffectDto>
                                  pddl_condition_effect_dto);

  kant_interfaces::msg::PddlAction
  action_dto_to_msg(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
};

} // namespace parser
} // namespace knowledge_base
} // namespace kant

#endif
