
#ifndef KANT_PDDL_TYPE_DAO_HPP
#define KANT_PDDL_TYPE_DAO_HPP

#include <string>

#include "kant_dto/pddl_type_dto.hpp"

namespace kant {
namespace dao {
namespace dao_interface {

class PddlTypeDao {
public:
  virtual std::shared_ptr<kant::dto::PddlTypeDto>
  get(std::string type_name) = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> get_all() = 0;

  virtual bool save(std::shared_ptr<kant::dto::PddlTypeDto> pdd_dto) = 0;
  virtual bool update(std::shared_ptr<kant::dto::PddlTypeDto> pdd_dto) = 0;
  virtual bool save_update(std::shared_ptr<kant::dto::PddlTypeDto> pdd_dto) = 0;

  virtual bool delete_one(std::shared_ptr<kant::dto::PddlTypeDto> pdd_dto) = 0;
  virtual bool delete_all() = 0;
};

} // namespace dao_interface
} // namespace dao
} // namespace kant

#endif
