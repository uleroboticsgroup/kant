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

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"

using namespace kant::dto;

PddlPropositionDto::PddlPropositionDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects)
    : PddlPropositionDto(predicate, objects, false) {}

PddlPropositionDto::PddlPropositionDto(
    std::shared_ptr<PddlPredicateDto> predicate,
    std::vector<std::shared_ptr<PddlObjectDto>> objects, bool is_goal) {
  this->set_predicate(predicate);
  this->set_objects(objects);
  this->set_is_goal(is_goal);
}

std::shared_ptr<PddlPredicateDto> PddlPropositionDto::get_predicate() {
  return this->predicate;
}

void PddlPropositionDto::set_predicate(
    std::shared_ptr<PddlPredicateDto> predicate) {
  this->predicate = predicate;
}

std::vector<std::shared_ptr<PddlObjectDto>> PddlPropositionDto::get_objects() {
  return this->objects;
}

void PddlPropositionDto::set_objects(
    std::vector<std::shared_ptr<PddlObjectDto>> objects) {
  this->objects = objects;
}

bool PddlPropositionDto::get_is_goal() { return this->is_goal; }

void PddlPropositionDto::set_is_goal(bool is_goal) { this->is_goal = is_goal; }

std::string PddlPropositionDto::to_string() {

  std::string result = "(" + this->predicate->get_name();

  for (unsigned i = 0; i < this->objects.size(); i++) {
    result += " " + this->objects.at(i)->get_name();
  }

  result += ")";

  return result;
}

bool PddlPropositionDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlPropositionDto>(dto);

  if (other == nullptr) {
    return false;
  }

  if (!other->get_predicate()->equals(this->predicate)) {
    return false;
  }

  if (other->get_objects().size() != this->objects.size()) {
    return false;
  }

  if (other->get_is_goal() != this->is_goal) {
    return false;
  }

  for (unsigned i = 0; i < this->objects.size(); i++) {
    if (!other->get_objects().at(i)->equals(this->objects.at(i))) {
      return false;
    }
  }

  return true;
}
