
#ifndef KANT_PDDL_OBJECT_DAO_HPP
#define KANT_PDDL_OBJECT_DAO_HPP

#include <string>

#include "kant_dto/pddl_object_dto.hpp"

namespace kant {
namespace dao {
namespace dao_interface {

class PddlObjectDao {
public:
  virtual std::shared_ptr<kant::dto::PddlObjectDto>
  get(std::string object_name) = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> get_all() = 0;

  virtual bool save(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool update(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool
  save_update(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;

  virtual bool
  delete_one(std::shared_ptr<kant::dto::PddlObjectDto> pdd_dto) = 0;
  virtual bool delete_all() = 0;
};

} // namespace dao_interface
} // namespace dao
} // namespace kant

#endif
