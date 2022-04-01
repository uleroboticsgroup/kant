
#ifndef PDDL_CONDITION_EFFECT_DTO_HPP
#define PDDL_CONDITION_EFFECT_DTO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"

namespace kant {
namespace dto {

const std::string AT_START = "at start";
const std::string AT_END = "at end";
const std::string OVER_ALL = "over all";

class PddlConditionEffectDto : public PddlPropositionDto {

private:
  std::string time;
  bool is_negative;

public:
  PddlConditionEffectDto(std::shared_ptr<PddlPredicateDto> predicate,
                         std::vector<std::shared_ptr<PddlObjectDto>> objects);
  PddlConditionEffectDto(std::shared_ptr<PddlPredicateDto> predicate,
                         std::vector<std::shared_ptr<PddlObjectDto>> objects,
                         bool is_negative);
  PddlConditionEffectDto(std::shared_ptr<PddlPredicateDto> predicate,
                         std::vector<std::shared_ptr<PddlObjectDto>> objects,
                         std::string time);
  PddlConditionEffectDto(std::shared_ptr<PddlPredicateDto> predicate,
                         std::vector<std::shared_ptr<PddlObjectDto>> objects,
                         bool is_negative, std::string time);

  std::string get_time();
  void set_time(std::string time);

  bool get_is_negative();
  void set_is_negative(bool is_negative);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif