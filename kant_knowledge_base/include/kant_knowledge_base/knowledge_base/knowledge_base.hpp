
#ifndef KANT_KNOWLEDGE_BASE_HPP
#define KANT_KNOWLEDGE_BASE_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "kant_dto/pddl_action_dto.hpp"
#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_proposition_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"

namespace kant {
namespace knowledge_base {
namespace knowledge_base {

class KnowledgeBase {
private:
  std::map<std::string, std::shared_ptr<kant::dto::PddlTypeDto>> types;
  std::map<std::string, std::shared_ptr<kant::dto::PddlObjectDto>> objects;
  std::map<std::string, std::shared_ptr<kant::dto::PddlPredicateDto>>
      predicates;
  std::map<std::string, std::shared_ptr<kant::dto::PddlActionDto>> actions;
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> propositions;
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>> goals;

public:
  std::shared_ptr<kant::dto::PddlTypeDto> get_type(std::string type_name);
  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> get_all_types();
  bool save_type(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);
  bool delete_type(std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto);
  bool delete_all_types();

  std::shared_ptr<kant::dto::PddlObjectDto> get_object(std::string object_name);
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> get_all_objects();
  bool save_object(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool delete_object(std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto);
  bool delete_all_objects();

  std::shared_ptr<kant::dto::PddlPredicateDto>
  get_predicate(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
  get_all_predicates();
  bool save_predicate(
      std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool delete_predicate(
      std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto);
  bool delete_all_predicates();

  std::shared_ptr<kant::dto::PddlActionDto> get_action(std::string action_name);
  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> get_all_actions();
  bool prepare_condition_effect_to_save(
      std::shared_ptr<kant::dto::PddlConditionEffectDto> pddl_condi_effect_dto,
      std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool save_action(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool delete_action(std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto);
  bool delete_all_actions();

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_propositions(std::string predicate_name);
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_propositions_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_propositions_no_goals();
  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
  get_all_propositions();
  bool check_proposition(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool save_proposition(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool delete_proposition(
      std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto);
  bool delete_all_propositions();
};

} // namespace knowledge_base
} // namespace knowledge_base
} // namespace kant

#endif
