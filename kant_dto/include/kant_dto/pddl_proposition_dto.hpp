
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