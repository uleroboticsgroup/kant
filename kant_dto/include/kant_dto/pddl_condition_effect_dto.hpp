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