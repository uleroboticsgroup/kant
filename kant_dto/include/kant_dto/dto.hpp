
#ifndef PDDL_DTO_HPP
#define PDDL_DTO_HPP

#include <memory>
#include <string>

namespace kant {
namespace dto {

class Dto {

public:
  virtual ~Dto() {}
  virtual std::string to_string() = 0;
  virtual bool equals(std::shared_ptr<Dto> dto) = 0;
};

} // namespace dto
} // namespace kant

#endif