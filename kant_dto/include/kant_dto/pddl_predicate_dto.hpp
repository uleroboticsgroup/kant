
#ifndef PDDL_PREDICATE_DTO_HPP
#define PDDL_PREDICATE_DTO_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

namespace kant {
namespace dto {

class PddlPredicateDto : public Dto {

private:
  std::string name;
  std::vector<std::shared_ptr<PddlTypeDto>> types;

public:
  PddlPredicateDto(std::string name,
                   std::vector<std::shared_ptr<PddlTypeDto>> types);
  PddlPredicateDto(std::string name);

  std::string get_name();
  void set_name(std::string name);

  std::vector<std::shared_ptr<PddlTypeDto>> get_types();
  void set_types(std::vector<std::shared_ptr<PddlTypeDto>> types);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif