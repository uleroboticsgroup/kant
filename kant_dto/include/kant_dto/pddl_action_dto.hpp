
#ifndef PDDL_ACTION_DTO_HPP
#define PDDL_ACTION_DTO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"

namespace kant {
namespace dto {

class PddlActionDto : public Dto {

private:
  std::string name;
  std::vector<std::shared_ptr<PddlObjectDto>> parameters;
  std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions;
  std::vector<std::shared_ptr<PddlConditionEffectDto>> effects;
  bool durative;
  int duration;

public:
  PddlActionDto(std::string name);
  PddlActionDto(std::string name,
                std::vector<std::shared_ptr<PddlObjectDto>> parameters,
                std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions,
                std::vector<std::shared_ptr<PddlConditionEffectDto>> effects);
  PddlActionDto(std::string name,
                std::vector<std::shared_ptr<PddlObjectDto>> parameters,
                std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions,
                std::vector<std::shared_ptr<PddlConditionEffectDto>> effects,
                bool durative);

  std::string get_name();
  void set_name(std::string name);

  std::vector<std::shared_ptr<PddlObjectDto>> get_parameters();
  void set_parameters(std::vector<std::shared_ptr<PddlObjectDto>> parameters);

  std::vector<std::shared_ptr<PddlConditionEffectDto>> get_conditions();
  void set_conditions(
      std::vector<std::shared_ptr<PddlConditionEffectDto>> conditions);

  std::vector<std::shared_ptr<PddlConditionEffectDto>> get_effects();
  void
  set_effects(std::vector<std::shared_ptr<PddlConditionEffectDto>> effects);

  bool get_durative();
  void set_durative(bool durative);

  int get_duration();
  void set_duration(int duration);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif