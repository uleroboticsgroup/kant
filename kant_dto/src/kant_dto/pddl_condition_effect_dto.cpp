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

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"

using namespace kant::dto;

// CONSTRUCTORS
PddlConditionEffectDto::PddlConditionEffectDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects)

    : PddlConditionEffectDto(predicate, objects, false) {}

PddlConditionEffectDto::PddlConditionEffectDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects, bool is_negative)
    : PddlPropositionDto(predicate, objects) {

  this->set_is_negative(is_negative);
  this->set_time("");
}

PddlConditionEffectDto::PddlConditionEffectDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects, std::string time)
    : PddlPropositionDto(predicate, objects) {

  this->set_is_negative(false);
  this->set_time(time);
}

PddlConditionEffectDto::PddlConditionEffectDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects, bool is_negative,
    std::string time)
    : PddlPropositionDto(predicate, objects) {

  this->set_is_negative(is_negative);
  this->set_time(time);
}

// GETTERS AND SETTERS
std::string PddlConditionEffectDto::get_time() { return this->time; }

void PddlConditionEffectDto::set_time(std::string time) { this->time = time; }

bool PddlConditionEffectDto::get_is_negative() { return this->is_negative; }

void PddlConditionEffectDto::set_is_negative(bool is_negative) {
  this->is_negative = is_negative;
}

// TO_STRING AND EQUALS
std::string PddlConditionEffectDto::to_string() {

  std::string result = "(" + this->get_predicate()->get_name();

  for (unsigned i = 0; i < this->get_objects().size(); i++) {
    result += " ?" + this->get_objects().at(i)->get_name();
  }

  result += ")";

  if (this->is_negative) {
    result = "(not " + result + ")";
  }

  if (!this->time.empty()) {
    result = "(" + this->time + " " + result + ")";
  }

  return result;
}

bool PddlConditionEffectDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlConditionEffectDto>(dto);

  if (other == nullptr) {
    return false;
  }

  if (!PddlPropositionDto::equals(other)) {
    return false;
  }

  if (!other->get_is_negative() != this->is_negative) {
    return false;
  }

  if (other->get_time() != this->time) {
    return false;
  }

  return true;
}
