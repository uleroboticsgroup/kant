
#ifndef PDDL_OBJECT_DTO_HPP
#define PDDL_OBJECT_DTO_HPP

#include <memory>
#include <string>

#include "kant_dto/dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

namespace kant {
namespace dto {

class PddlObjectDto : public Dto {

private:
  std::shared_ptr<PddlTypeDto> type;
  std::string name;

public:
  PddlObjectDto(std::shared_ptr<PddlTypeDto> type, std::string name);

  std::shared_ptr<PddlTypeDto> get_type();
  void set_type(std::shared_ptr<PddlTypeDto> type);

  std::string get_name();
  void set_name(std::string name);

  std::string to_string();
  bool equals(std::shared_ptr<Dto> dto);
};

} // namespace dto
} // namespace kant

#endif