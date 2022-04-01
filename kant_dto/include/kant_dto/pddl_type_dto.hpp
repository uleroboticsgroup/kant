
#ifndef PDDL_TYPE_DTO_HPP
#define PDDL_TYPE_DTO_HPP

#include <memory>
#include <string>

#include "kant_dto/dto.hpp"

namespace kant {
namespace dto {

class PddlTypeDto : public Dto {

private:
  std::string name;

public:
  PddlTypeDto(std::string name);

  std::string get_name();
  void set_name(std::string name);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif