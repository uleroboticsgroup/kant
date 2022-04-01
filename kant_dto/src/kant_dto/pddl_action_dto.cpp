
#include <memory>
#include <string>

#include "kant_dto/pddl_action_dto.hpp"
#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"

using namespace kant::dto;

// CONTRUCTORS
PddlActionDto::PddlActionDto(std::string name)
    : PddlActionDto(name, {}, {}, {}, true) {}

PddlActionDto::PddlActionDto(
    std::string name, std::vector<std::shared_ptr<PddlObjectDto>> parameters,
    std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions,
    std::vector<std::shared_ptr<PddlConditionEffectDto>> effects)
    : PddlActionDto(name, parameters, conditions, effects, true) {}

PddlActionDto::PddlActionDto(
    std::string name, std::vector<std::shared_ptr<PddlObjectDto>> parameters,
    std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions,
    std::vector<std::shared_ptr<PddlConditionEffectDto>> effects,
    bool durative) {

  this->set_name(name);
  this->set_parameters(parameters);
  this->set_conditions(conditions);
  this->set_effects(effects);
  this->set_durative(durative);
  this->set_duration(10);
}

// GETTERS AND SETTERS
std::string PddlActionDto::get_name() { return this->name; }

void PddlActionDto::set_name(std::string name) { this->name = name; }

std::vector<std::shared_ptr<PddlObjectDto>> PddlActionDto::get_parameters() {
  return this->parameters;
}

void PddlActionDto::set_parameters(
    std::vector<std::shared_ptr<PddlObjectDto>> parameters) {
  this->parameters = parameters;
}

std::vector<std::shared_ptr<PddlConditionEffectDto>>
PddlActionDto::get_conditions() {
  return this->conditions;
}

void PddlActionDto::set_conditions(
    std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions) {
  this->conditions = conditions;
}

std::vector<std::shared_ptr<PddlConditionEffectDto>>
PddlActionDto::get_effects() {
  return this->effects;
}

void PddlActionDto::set_effects(
    std::vector<std::shared_ptr<PddlConditionEffectDto>> effects) {
  this->effects = effects;
}

bool PddlActionDto::get_durative() { return this->durative; }

void PddlActionDto::set_durative(bool durative) { this->durative = durative; }

int PddlActionDto::get_duration() { return this->duration; }

void PddlActionDto::set_duration(int duration) { this->duration = duration; }

// TO_STRING AND EQUALS
std::string PddlActionDto::to_string() {
  std::string result = "(:";

  // durative
  if (this->durative) {
    result += "durative-";
  }
  result += "action " + this->name;

  // parameters
  result += "\n\t:parameters (";
  for (std::shared_ptr<PddlObjectDto> parameter : this->parameters) {
    result += " ?" + parameter->get_name() + " - " +
              parameter->get_type()->get_name();
  }
  result += ")";

  // duration
  if (this->durative) {
    result +=
        "\n\t:duration (= ?duration " + std::to_string(this->duration) + ")";
  }

  // conditions
  if (this->durative) {
    result += "\n\t:condition (and";
  } else {
    result += "\n\t:precondition (and";
  }

  for (std::shared_ptr<PddlConditionEffectDto> condition : this->conditions) {
    result += "\n\t\t" + condition->to_string();
  }
  result += "\n\t)";

  // effects
  result += "\n\t:effect (and";

  for (std::shared_ptr<PddlConditionEffectDto> effect : this->effects) {
    result += "\n\t\t" + effect->to_string();
  }
  result += "\n\t)";

  result += "\n)";

  return result;
}

bool PddlActionDto::equals(std::shared_ptr<Dto> dto) {
  auto other = std::dynamic_pointer_cast<PddlActionDto>(dto);

  if (other == nullptr) {
    return false;
  }

  return (other->get_name() == this->get_name());
}
