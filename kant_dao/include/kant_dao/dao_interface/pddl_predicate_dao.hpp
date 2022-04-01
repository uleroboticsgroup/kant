
#ifndef KANT_PDDL_PREDICATE_DAO_HPP
#define KANT_PDDL_PREDICATE_DAO_HPP

#include <string>
#include <vector>

#include "kant_dto/pddl_predicate_dto.hpp"

namespace kant {
namespace dao {
namespace dao_interface {

class PddlPredicateDao {
public:
  virtual std::shared_ptr<kant::dto::PddlPredicateDto>
  get(std::string predicate_name) = 0;

  virtual bool save(std::shared_ptr<kant::dto::PddlPredicateDto> pdd_dto) = 0;
  virtual bool update(std::shared_ptr<kant::dto::PddlPredicateDto> pdd_dto) = 0;
  virtual bool
  save_update(std::shared_ptr<kant::dto::PddlPredicateDto> pdd_dto) = 0;

  virtual bool
  delete_one(std::shared_ptr<kant::dto::PddlPredicateDto> pdd_dto) = 0;
  virtual bool delete_all() = 0;
};

} // namespace dao_interface
} // namespace dao
} // namespace kant

#endif
