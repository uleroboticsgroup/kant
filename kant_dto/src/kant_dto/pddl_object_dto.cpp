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

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

using namespace kant::dto;

PddlObjectDto::PddlObjectDto(std::shared_ptr<PddlTypeDto> type,
                             std::string name) {
  this->set_type(type);
  this->set_name(name);
}

std::shared_ptr<PddlTypeDto> PddlObjectDto::get_type() { return this->type; }

void PddlObjectDto::set_type(std::shared_ptr<PddlTypeDto> type) {
  this->type = type;
}

std::string PddlObjectDto::get_name() { return this->name; }

void PddlObjectDto::set_name(std::string name) { this->name = name; }

std::string PddlObjectDto::to_string() {
  return this->get_name() + " - " + this->type->get_name();
}

bool PddlObjectDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlObjectDto>(dto);

  if (other == nullptr) {
    return false;
  }

  return (other->get_name() == this->get_name());
}
