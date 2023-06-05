// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef KANT_KNOWLEDGE_BASE_NODE_HPP
#define KANT_KNOWLEDGE_BASE_NODE_HPP

#include <memory>

#include "simple_node/node.hpp"
#include "std_srvs/srv/empty.hpp"

#include "kant_msgs/srv/get_pddl_action.hpp"
#include "kant_msgs/srv/get_pddl_object.hpp"
#include "kant_msgs/srv/get_pddl_predicate.hpp"
#include "kant_msgs/srv/get_pddl_proposition.hpp"
#include "kant_msgs/srv/get_pddl_type.hpp"
#include "kant_msgs/srv/update_pddl_action.hpp"
#include "kant_msgs/srv/update_pddl_object.hpp"
#include "kant_msgs/srv/update_pddl_predicate.hpp"
#include "kant_msgs/srv/update_pddl_proposition.hpp"
#include "kant_msgs/srv/update_pddl_type.hpp"

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

  rclcpp::Service<kant_msgs::srv::GetPddlType>::SharedPtr get_types_service;
  rclcpp::Service<kant_msgs::srv::UpdatePddlType>::SharedPtr
      update_type_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_types_service;

  rclcpp::Service<kant_msgs::srv::GetPddlObject>::SharedPtr get_objects_service;
  rclcpp::Service<kant_msgs::srv::UpdatePddlObject>::SharedPtr
      update_object_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_objects_service;

  rclcpp::Service<kant_msgs::srv::GetPddlPredicate>::SharedPtr
      get_predicates_service;
  rclcpp::Service<kant_msgs::srv::UpdatePddlPredicate>::SharedPtr
      update_predicate_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
      delete_all_predicates_service;

  rclcpp::Service<kant_msgs::srv::GetPddlProposition>::SharedPtr
      get_propositions_service;
  rclcpp::Service<kant_msgs::srv::UpdatePddlProposition>::SharedPtr
      update_proposition_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
      delete_all_propositions_service;

  rclcpp::Service<kant_msgs::srv::GetPddlAction>::SharedPtr get_actions_service;
  rclcpp::Service<kant_msgs::srv::UpdatePddlAction>::SharedPtr
      update_action_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_all_actions_service;

public:
  KnowledgeBaseNode();

  void
  get_types(const std::shared_ptr<kant_msgs::srv::GetPddlType::Request> request,
            std::shared_ptr<kant_msgs::srv::GetPddlType::Response> response);
  void update_type(
      const std::shared_ptr<kant_msgs::srv::UpdatePddlType::Request> request,
      std::shared_ptr<kant_msgs::srv::UpdatePddlType::Response> response);
  void
  delete_all_types(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_objects(
      const std::shared_ptr<kant_msgs::srv::GetPddlObject::Request> request,
      std::shared_ptr<kant_msgs::srv::GetPddlObject::Response> response);
  void update_object(
      const std::shared_ptr<kant_msgs::srv::UpdatePddlObject::Request> request,
      std::shared_ptr<kant_msgs::srv::UpdatePddlObject::Response> response);
  void delete_all_objects(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_predicates(
      const std::shared_ptr<kant_msgs::srv::GetPddlPredicate::Request> request,
      std::shared_ptr<kant_msgs::srv::GetPddlPredicate::Response> response);
  void update_predicate(
      const std::shared_ptr<kant_msgs::srv::UpdatePddlPredicate::Request>
          request,
      std::shared_ptr<kant_msgs::srv::UpdatePddlPredicate::Response> response);
  void delete_all_predicates(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_propositions(
      const std::shared_ptr<kant_msgs::srv::GetPddlProposition::Request>
          request,
      std::shared_ptr<kant_msgs::srv::GetPddlProposition::Response> response);
  void update_proposition(
      const std::shared_ptr<kant_msgs::srv::UpdatePddlProposition::Request>
          request,
      std::shared_ptr<kant_msgs::srv::UpdatePddlProposition::Response>
          response);
  void delete_all_propositions(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void get_actions(
      const std::shared_ptr<kant_msgs::srv::GetPddlAction::Request> request,
      std::shared_ptr<kant_msgs::srv::GetPddlAction::Response> response);
  void update_action(
      const std::shared_ptr<kant_msgs::srv::UpdatePddlAction::Request> request,
      std::shared_ptr<kant_msgs::srv::UpdatePddlAction::Response> response);
  void delete_all_actions(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

} // namespace knowledge_base
} // namespace knowledge_base
} // namespace kant

#endif
