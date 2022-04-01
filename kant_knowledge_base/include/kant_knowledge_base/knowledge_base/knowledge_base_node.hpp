
#ifndef KANT_KNOWLEDGE_BASE_NODE_HPP
#define KANT_KNOWLEDGE_BASE_NODE_HPP

#include <memory>

#include "simple_node/node.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_interfaces/srv/get_pddl_action.hpp"
#include "kant_interfaces/srv/get_pddl_object.hpp"
#include "kant_interfaces/srv/get_pddl_predicate.hpp"
#include "kant_interfaces/srv/get_pddl_proposition.hpp"
#include "kant_interfaces/srv/get_pddl_type.hpp"
#include "kant_interfaces/srv/update_pddl_action.hpp"
#include "kant_interfaces/srv/update_pddl_object.hpp"
#include "kant_interfaces/srv/update_pddl_predicate.hpp"
#include "kant_interfaces/srv/update_pddl_proposition.hpp"
#include "kant_interfaces/srv/update_pddl_type.hpp"

#include "kant_knowledge_base/knowledge_base/knowledge_base.hpp"
#include "kant_knowledge_base/parser/dto_msg_parser.hpp"
#include "kant_knowledge_base/parser/msg_dto_parser.hpp"

namespace kant {
namespace knowledge_base {
namespace knowledge_base {

class KnowledgeBaseNode : public rclcpp::Node {
private:
  std::unique_ptr<kant::knowledge_base::parser::DtoMsgParser> dto_msg_parser;
  std::unique_ptr<kant::knowledge_base::parser::MsgDtoParser> msg_dto_parser;
  std::unique_ptr<kant::knowledge_base::knowledge_base::KnowledgeBase>
      knowledge_base;

  rclcpp::Service<kant_interfaces::srv::GetPddlType>::SharedPtr
      get_types_service;
  rclcpp::Service<kant_interfaces::srv::UpdatePddlType>::SharedPtr
      update_type_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_types_service;

  rclcpp::Service<kant_interfaces::srv::GetPddlObject>::SharedPtr
      get_objects_service;
  rclcpp::Service<kant_interfaces::srv::UpdatePddlObject>::SharedPtr
      update_object_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_objects_service;

  rclcpp::Service<kant_interfaces::srv::GetPddlPredicate>::SharedPtr
      get_predicates_service;
  rclcpp::Service<kant_interfaces::srv::UpdatePddlPredicate>::SharedPtr
      update_predicate_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
      delete_all_predicates_service;

  rclcpp::Service<kant_interfaces::srv::GetPddlProposition>::SharedPtr
      get_propositions_service;
  rclcpp::Service<kant_interfaces::srv::UpdatePddlProposition>::SharedPtr
      update_proposition_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
      delete_all_propositions_service;

  rclcpp::Service<kant_interfaces::srv::GetPddlAction>::SharedPtr
      get_actions_service;
  rclcpp::Service<kant_interfaces::srv::UpdatePddlAction>::SharedPtr
      update_action_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_actions_service;

public:
  KnowledgeBaseNode();

  void get_types(
      const std::shared_ptr<kant_interfaces::srv::GetPddlType::Request> request,
      std::shared_ptr<kant_interfaces::srv::GetPddlType::Response> response);
  void update_type(
      const std::shared_ptr<kant_interfaces::srv::UpdatePddlType::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::UpdatePddlType::Response> response);
  void
  delete_all_types(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_objects(
      const std::shared_ptr<kant_interfaces::srv::GetPddlObject::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::GetPddlObject::Response> response);
  void update_object(
      const std::shared_ptr<kant_interfaces::srv::UpdatePddlObject::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::UpdatePddlObject::Response>
          response);
  void delete_all_objects(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_predicates(
      const std::shared_ptr<kant_interfaces::srv::GetPddlPredicate::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::GetPddlPredicate::Response>
          response);
  void update_predicate(
      const std::shared_ptr<kant_interfaces::srv::UpdatePddlPredicate::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::UpdatePddlPredicate::Response>
          response);
  void delete_all_predicates(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_propositions(
      const std::shared_ptr<kant_interfaces::srv::GetPddlProposition::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::GetPddlProposition::Response>
          response);
  void update_proposition(
      const std::shared_ptr<
          kant_interfaces::srv::UpdatePddlProposition::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::UpdatePddlProposition::Response>
          response);
  void delete_all_propositions(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_actions(
      const std::shared_ptr<kant_interfaces::srv::GetPddlAction::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::GetPddlAction::Response> response);
  void update_action(
      const std::shared_ptr<kant_interfaces::srv::UpdatePddlAction::Request>
          request,
      std::shared_ptr<kant_interfaces::srv::UpdatePddlAction::Response>
          response);
  void delete_all_actions(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

} // namespace knowledge_base
} // namespace knowledge_base
} // namespace kant

#endif
