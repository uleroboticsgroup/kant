// Copyright (C) 2023  Miguel Ángel González Santamarta

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

#ifndef PDDL_PROPOSITION_DTO_HPP
#define PDDL_PROPOSITION_DTO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"

namespace kant {
namespace dto {

class PddlPropositionDto : public Dto {

private:
  std::shared_ptr<PddlPredicateDto> predicate;
  std::vector<std::shared_ptr<PddlObjectDto>> objects;
  bool is_goal;

public:
  PddlPropositionDto(std::shared_ptr<PddlPredicateDto> predicate,
                     std::vector<std::shared_ptr<PddlObjectDto>> objects);
  PddlPropositionDto(std::shared_ptr<PddlPredicateDto> predicate,
                     std::vector<std::shared_ptr<PddlObjectDto>> objects,
                     bool is_goal);

  std::shared_ptr<PddlPredicateDto> get_predicate();
  void set_predicate(std::shared_ptr<PddlPredicateDto> predicate);

  std::vector<std::shared_ptr<PddlObjectDto>> get_objects();
  void set_objects(std::vector<std::shared_ptr<PddlObjectDto>> objects);

  bool get_is_goal();
  void set_is_goal(bool is_goal);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif