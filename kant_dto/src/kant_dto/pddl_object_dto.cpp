
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
