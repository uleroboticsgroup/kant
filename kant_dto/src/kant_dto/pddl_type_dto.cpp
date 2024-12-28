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

#include "kant_dto/pddl_type_dto.hpp"
#include "kant_dto/dto.hpp"

using namespace kant::dto;

PddlTypeDto::PddlTypeDto(std::string name) { this->set_name(name); }

std::string PddlTypeDto::get_name() { return this->name; }

void PddlTypeDto::set_name(std::string name) { this->name = name; }

std::string PddlTypeDto::to_string() { return this->name; }

bool PddlTypeDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlTypeDto>(dto);

  if (other == nullptr) {
    return false;
  }

  return (other->get_name() == this->get_name());
}
