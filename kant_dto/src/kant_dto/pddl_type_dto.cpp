
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
