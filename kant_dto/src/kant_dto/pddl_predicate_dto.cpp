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
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

using namespace kant::dto;

PddlPredicateDto::PddlPredicateDto(std::string name)
    : PddlPredicateDto(name, {}) {}

PddlPredicateDto::PddlPredicateDto(
    std::string name, std::vector<std::shared_ptr<PddlTypeDto>> types) {
  this->set_name(name);
  this->set_types(types);
}

std::string PddlPredicateDto::get_name() { return this->name; }

void PddlPredicateDto::set_name(std::string name) { this->name = name; }

std::vector<std::shared_ptr<PddlTypeDto>> PddlPredicateDto::get_types() {
  return this->types;
}

void PddlPredicateDto::set_types(
    std::vector<std::shared_ptr<PddlTypeDto>> types) {
  this->types = types;
}

std::string PddlPredicateDto::to_string() {
  std::string result = "(" + this->get_name();

  for (unsigned i = 0; i < this->types.size(); i++) {
    std::string type_name = this->types.at(i)->get_name();
    std::string first_char(1, type_name[0]);
    result +=
        " ?" + first_char + std::to_string(i) + std::string(" - ") + type_name;
  }

  result += ")";

  return result;
}

bool PddlPredicateDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlPredicateDto>(dto);

  if (other == nullptr) {
    return false;
  }

  return (other->get_name() == this->get_name());
}
