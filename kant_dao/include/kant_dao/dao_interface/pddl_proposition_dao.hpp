
#ifndef KANT_PDDL_PROPOSITION_DAO_HPP
#define KANT_PDDL_PROPOSITION_DAO_HPP

#include <string>
#include <vector>

#include "kant_dto/pddl_proposition_dto.hpp"

namespace kant {
namespace dao {
namespace dao_interface {

class PddlPropositionDao {
public:
  virtual std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_by_predicate(std::string predicate_name) = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_goals() = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_no_goals() = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_all() = 0;

  virtual bool save(std::shared_ptr<kant::dto::PddlPropositionDto> pdd_dto) = 0;
  virtual bool
  update(std::shared_ptr<kant::dto::PddlPropositionDto> pdd_dto) = 0;
  virtual bool
  save_update(std::shared_ptr<kant::dto::PddlPropositionDto> pdd_dto) = 0;

  virtual bool
  delete_one(std::shared_ptr<kant::dto::PddlPropositionDto> pdd_dto) = 0;
  virtual bool delete_all() = 0;
};

} // namespace dao_interface
} // namespace dao
} // namespace kant

#endif
