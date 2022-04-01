
#include <functional>
#include <vector>

#include "kant_interfaces/msg/pddl_action.hpp"
#include "kant_interfaces/msg/pddl_object.hpp"
#include "kant_interfaces/msg/pddl_proposition.hpp"
#include "kant_interfaces/msg/pddl_type.hpp"
#include "kant_interfaces/msg/update_knowledge.hpp"

#include "kant_dto/pddl_proposition_dto.hpp"

#include "kant_knowledge_base/knowledge_base/knowledge_base_node.hpp"

using namespace kant::knowledge_base::knowledge_base;

KnowledgeBaseNode::KnowledgeBaseNode() : Node("knowledge_base_node") {
  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();
  this->knowledge_base =
      std::make_unique<kant::knowledge_base::knowledge_base::KnowledgeBase>();

  this->get_types_service =
      this->create_service<kant_interfaces::srv::GetPddlType>(
          "get_types", std::bind(&KnowledgeBaseNode::get_types, this,
                                 std::placeholders::_1, std::placeholders::_2));
  this->update_type_service =
      this->create_service<kant_interfaces::srv::UpdatePddlType>(
          "update_type",
          std::bind(&KnowledgeBaseNode::update_type, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->delete_all_types_service = this->create_service<std_srvs::srv::Empty>(
      "delete_all_types",
      std::bind(&KnowledgeBaseNode::delete_all_types, this,
                std::placeholders::_1, std::placeholders::_2));

  this->get_objects_service =
      this->create_service<kant_interfaces::srv::GetPddlObject>(
          "get_objects",
          std::bind(&KnowledgeBaseNode::get_objects, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->update_object_service =
      this->create_service<kant_interfaces::srv::UpdatePddlObject>(
          "update_object",
          std::bind(&KnowledgeBaseNode::update_object, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->delete_all_objects_service = this->create_service<std_srvs::srv::Empty>(
      "delete_all_objects",
      std::bind(&KnowledgeBaseNode::delete_all_objects, this,
                std::placeholders::_1, std::placeholders::_2));

  this->get_predicates_service =
      this->create_service<kant_interfaces::srv::GetPddlPredicate>(
          "get_predicates",
          std::bind(&KnowledgeBaseNode::get_predicates, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->update_predicate_service =
      this->create_service<kant_interfaces::srv::UpdatePddlPredicate>(
          "update_predicate",
          std::bind(&KnowledgeBaseNode::update_predicate, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->delete_all_predicates_service =
      this->create_service<std_srvs::srv::Empty>(
          "delete_all_predicates",
          std::bind(&KnowledgeBaseNode::delete_all_predicates, this,
                    std::placeholders::_1, std::placeholders::_2));

  this->get_propositions_service =
      this->create_service<kant_interfaces::srv::GetPddlProposition>(
          "get_propositions",
          std::bind(&KnowledgeBaseNode::get_propositions, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->update_proposition_service =
      this->create_service<kant_interfaces::srv::UpdatePddlProposition>(
          "update_proposition",
          std::bind(&KnowledgeBaseNode::update_proposition, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->delete_all_propositions_service =
      this->create_service<std_srvs::srv::Empty>(
          "delete_all_propositions",
          std::bind(&KnowledgeBaseNode::delete_all_propositions, this,
                    std::placeholders::_1, std::placeholders::_2));

  this->get_actions_service =
      this->create_service<kant_interfaces::srv::GetPddlAction>(
          "get_actions",
          std::bind(&KnowledgeBaseNode::get_actions, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->update_action_service =
      this->create_service<kant_interfaces::srv::UpdatePddlAction>(
          "update_action",
          std::bind(&KnowledgeBaseNode::update_action, this,
                    std::placeholders::_1, std::placeholders::_2));
  this->delete_all_actions_service = this->create_service<std_srvs::srv::Empty>(
      "delete_all_actions",
      std::bind(&KnowledgeBaseNode::delete_all_actions, this,
                std::placeholders::_1, std::placeholders::_2));
}

//***********************
// PDDL TYPES
//***********************
void KnowledgeBaseNode::get_types(
    const std::shared_ptr<kant_interfaces::srv::GetPddlType::Request> request,
    std::shared_ptr<kant_interfaces::srv::GetPddlType::Response> response) {

  if (!request->type_name.empty()) {

    auto pddl_type_dto = this->knowledge_base->get_type(request->type_name);

    if (pddl_type_dto != nullptr) {

      auto pddl_type_msg = this->dto_msg_parser->type_dto_to_msg(pddl_type_dto);

      response->pddl_types = {pddl_type_msg};
    }
  } else {

    auto pddl_type_dtos = this->knowledge_base->get_all_types();

    std::vector<kant_interfaces::msg::PddlType> pddl_type_msg_list;
    for (const auto &pddl_type_dto : pddl_type_dtos) {
      pddl_type_msg_list.push_back(
          this->dto_msg_parser->type_dto_to_msg(pddl_type_dto));
    }

    response->pddl_types = pddl_type_msg_list;
  }
}

void KnowledgeBaseNode::update_type(
    const std::shared_ptr<kant_interfaces::srv::UpdatePddlType::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::UpdatePddlType::Response> response) {

  bool succ = false;

  auto pddl_type_dto =
      this->msg_dto_parser->type_msg_to_dto(request->pddl_type);

  if (request->update_konwledge.update_type ==
      kant_interfaces::msg::UpdateKnowledge::SAVE) {
    succ = this->knowledge_base->save_type(pddl_type_dto);

  } else if (request->update_konwledge.update_type ==
             kant_interfaces::msg::UpdateKnowledge::DELETE) {
    succ = this->knowledge_base->delete_type(pddl_type_dto);
  }

  response->success = succ;
}

void KnowledgeBaseNode::delete_all_types(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;
  this->knowledge_base->delete_all_types();
}

//***********************
// PDDL OBJECTS
//***********************
void KnowledgeBaseNode::get_objects(
    const std::shared_ptr<kant_interfaces::srv::GetPddlObject::Request> request,
    std::shared_ptr<kant_interfaces::srv::GetPddlObject::Response> response) {

  if (!request->object_name.empty()) {

    auto pddl_object_dto =
        this->knowledge_base->get_object(request->object_name);

    if (pddl_object_dto != nullptr) {

      auto pddl_object_msg =
          this->dto_msg_parser->object_dto_to_msg(pddl_object_dto);

      response->pddl_objects = {pddl_object_msg};
    }
  } else {

    auto pddl_object_dtos = this->knowledge_base->get_all_objects();

    std::vector<kant_interfaces::msg::PddlObject> pddl_object_msg_list;
    for (const auto &pddl_object_dto : pddl_object_dtos) {
      pddl_object_msg_list.push_back(
          this->dto_msg_parser->object_dto_to_msg(pddl_object_dto));
    }

    response->pddl_objects = pddl_object_msg_list;
  }
}

void KnowledgeBaseNode::update_object(
    const std::shared_ptr<kant_interfaces::srv::UpdatePddlObject::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::UpdatePddlObject::Response>
        response) {

  bool succ = false;

  auto pddl_object_dto =
      this->msg_dto_parser->object_msg_to_dto(request->pddl_object);

  if (request->update_konwledge.update_type ==
      kant_interfaces::msg::UpdateKnowledge::SAVE) {
    succ = this->knowledge_base->save_object(pddl_object_dto);

  } else if (request->update_konwledge.update_type ==
             kant_interfaces::msg::UpdateKnowledge::DELETE) {
    succ = this->knowledge_base->delete_object(pddl_object_dto);
  }

  response->success = succ;
}

void KnowledgeBaseNode::delete_all_objects(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;
  this->knowledge_base->delete_all_objects();
}

//***********************
// PDDL PREDICATES
//***********************
void KnowledgeBaseNode::get_predicates(
    const std::shared_ptr<kant_interfaces::srv::GetPddlPredicate::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::GetPddlPredicate::Response>
        response) {

  if (!request->predicate_name.empty()) {

    auto pddl_predicate_dto =
        this->knowledge_base->get_predicate(request->predicate_name);

    if (pddl_predicate_dto != nullptr) {

      auto pddl_predicate_msg =
          this->dto_msg_parser->predicate_dto_to_msg(pddl_predicate_dto);

      response->pddl_predicates = {pddl_predicate_msg};
    }
  } else {

    auto pddl_predicate_dtos = this->knowledge_base->get_all_predicates();

    std::vector<kant_interfaces::msg::PddlPredicate> pddl_predicate_msg_list;
    for (const auto &pddl_predicate_dto : pddl_predicate_dtos) {
      pddl_predicate_msg_list.push_back(
          this->dto_msg_parser->predicate_dto_to_msg(pddl_predicate_dto));
    }

    response->pddl_predicates = pddl_predicate_msg_list;
  }
}

void KnowledgeBaseNode::update_predicate(
    const std::shared_ptr<kant_interfaces::srv::UpdatePddlPredicate::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::UpdatePddlPredicate::Response>
        response) {

  bool succ = false;

  auto pddl_predicate_dto =
      this->msg_dto_parser->predicate_msg_to_dto(request->pddl_predicate);

  if (request->update_konwledge.update_type ==
      kant_interfaces::msg::UpdateKnowledge::SAVE) {
    succ = this->knowledge_base->save_predicate(pddl_predicate_dto);

  } else if (request->update_konwledge.update_type ==
             kant_interfaces::msg::UpdateKnowledge::DELETE) {
    succ = this->knowledge_base->delete_predicate(pddl_predicate_dto);
  }

  response->success = succ;
}

void KnowledgeBaseNode::delete_all_predicates(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;
  this->knowledge_base->delete_all_predicates();
}

//***********************
// PDDL PROPOSITIONS
//***********************
void KnowledgeBaseNode::get_propositions(
    const std::shared_ptr<kant_interfaces::srv::GetPddlProposition::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::GetPddlProposition::Response>
        response) {

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dtos;

  if (request->get_type ==
      kant_interfaces::srv::GetPddlProposition::Request::ALL) {

    pddl_proposition_dtos = this->knowledge_base->get_all_propositions();

  } else if (request->get_type ==
             kant_interfaces::srv::GetPddlProposition::Request::GOALS) {

    pddl_proposition_dtos = this->knowledge_base->get_propositions_goals();

  } else if (request->get_type ==
             kant_interfaces::srv::GetPddlProposition::Request::NO_GOALS) {
    pddl_proposition_dtos = this->knowledge_base->get_propositions_no_goals();

  } else if (request->get_type ==
             kant_interfaces::srv::GetPddlProposition::Request::BY_PREDICATE) {

    pddl_proposition_dtos =
        this->knowledge_base->get_propositions(request->predicate_name);
  }

  std::vector<kant_interfaces::msg::PddlProposition> pddl_predicate_msg_list;
  for (const auto &pddl_proposition_dto : pddl_proposition_dtos) {
    pddl_predicate_msg_list.push_back(
        this->dto_msg_parser->proposition_dto_to_msg(pddl_proposition_dto));
  }

  response->pddl_propositions = pddl_predicate_msg_list;
}

void KnowledgeBaseNode::update_proposition(
    const std::shared_ptr<kant_interfaces::srv::UpdatePddlProposition::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::UpdatePddlProposition::Response>
        response) {

  bool succ = false;

  auto pddl_proposition_dto =
      this->msg_dto_parser->proposition_msg_to_dto(request->pddl_proposition);

  if (request->update_konwledge.update_type ==
      kant_interfaces::msg::UpdateKnowledge::SAVE) {
    succ = this->knowledge_base->save_proposition(pddl_proposition_dto);

  } else if (request->update_konwledge.update_type ==
             kant_interfaces::msg::UpdateKnowledge::DELETE) {
    succ = this->knowledge_base->delete_proposition(pddl_proposition_dto);
  }
  response->success = succ;
}

void KnowledgeBaseNode::delete_all_propositions(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;
  this->knowledge_base->delete_all_propositions();
}

//***********************
// PDDL ACTIONS
//***********************
void KnowledgeBaseNode::get_actions(
    const std::shared_ptr<kant_interfaces::srv::GetPddlAction::Request> request,
    std::shared_ptr<kant_interfaces::srv::GetPddlAction::Response> response) {

  if (!request->action_name.empty()) {

    auto pddl_action_dto =
        this->knowledge_base->get_action(request->action_name);

    if (pddl_action_dto != nullptr) {

      auto pddl_action_msg =
          this->dto_msg_parser->action_dto_to_msg(pddl_action_dto);

      response->pddl_actions = {pddl_action_msg};
    }
  } else {

    auto pddl_action_dtos = this->knowledge_base->get_all_actions();

    std::vector<kant_interfaces::msg::PddlAction> pddl_action_msg_list;

    for (const auto &pddl_action_dto : pddl_action_dtos) {
      pddl_action_msg_list.push_back(
          this->dto_msg_parser->action_dto_to_msg(pddl_action_dto));

      response->pddl_actions = pddl_action_msg_list;
    }
  }
}

void KnowledgeBaseNode::update_action(
    const std::shared_ptr<kant_interfaces::srv::UpdatePddlAction::Request>
        request,
    std::shared_ptr<kant_interfaces::srv::UpdatePddlAction::Response>
        response) {

  bool succ = false;

  auto pddl_action_dto =
      this->msg_dto_parser->action_msg_to_dto(request->pddl_action);

  if (request->update_konwledge.update_type ==
      kant_interfaces::msg::UpdateKnowledge::SAVE) {
    succ = this->knowledge_base->save_action(pddl_action_dto);

  } else if (request->update_konwledge.update_type ==
             kant_interfaces::msg::UpdateKnowledge::DELETE) {
    succ = this->knowledge_base->delete_action(pddl_action_dto);
  }

  response->success = succ;
}

void KnowledgeBaseNode::delete_all_actions(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;
  this->knowledge_base->delete_all_actions();
}
