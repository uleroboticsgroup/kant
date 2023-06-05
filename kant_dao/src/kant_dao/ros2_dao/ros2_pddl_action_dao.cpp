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

#include "kant_msgs/msg/update_knowledge.hpp"

#include "kant_dao/ros2_dao/ros2_pddl_action_dao.hpp"

using namespace kant::dao::ros2_dao;

Ros2PddlActionDao::Ros2PddlActionDao(simple_node::Node *node) {

  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();

  // srv clients
  this->get_client =
      node->create_client<kant_msgs::srv::GetPddlAction>("get_actions");

  this->update_client =
      node->create_client<kant_msgs::srv::UpdatePddlAction>("update_action");

  this->delete_all_client =
      node->create_client<std_srvs::srv::Empty>("delete_all_actions");
}

std::vector<std::shared_ptr<kant::dto::PddlActionDto>>
Ros2PddlActionDao::ros2_get(std::string action_name) {

  auto request = std::make_shared<kant_msgs::srv::GetPddlAction::Request>();

  request->action_name = action_name;

  this->get_client->wait_for_service();
  auto future = this->get_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  std::vector<std::shared_ptr<kant::dto::PddlActionDto>> pddl_action_dto_list;

  for (const auto &pddl_action_msg : result->pddl_actions) {
    auto pddl_action_dto =
        this->msg_dto_parser->action_msg_to_dto(pddl_action_msg);
    pddl_action_dto_list.push_back(pddl_action_dto);
  }

  return pddl_action_dto_list;
}

bool Ros2PddlActionDao::ros2_update(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto,
    int update_type) {

  auto request = std::make_shared<kant_msgs::srv::UpdatePddlAction::Request>();

  request->pddl_action =
      this->dto_msg_parser->action_dto_to_msg(pddl_action_dto);
  request->update_konwledge.update_type = update_type;

  this->update_client->wait_for_service();
  auto future = this->update_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  return result->success;
}

void Ros2PddlActionDao::ros2_delete_all() {

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  this->delete_all_client->wait_for_service();
  auto future = this->delete_all_client->async_send_request(request);
  future.wait();
}

std::shared_ptr<kant::dto::PddlActionDto>
Ros2PddlActionDao::get(std::string action_name) {

  auto pddl_action_dto_list = this->ros2_get(action_name);

  if (pddl_action_dto_list.size() == 1) {
    return pddl_action_dto_list.at(0);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlActionDto>>
Ros2PddlActionDao::get_all() {

  auto pddl_action_dto_list = this->ros2_get("");
  return pddl_action_dto_list;
}

bool Ros2PddlActionDao::save(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {

  if (this->get(pddl_action_dto->get_name()) == nullptr) {
    return this->ros2_update(pddl_action_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlActionDao::update(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  if (this->get(pddl_action_dto->get_name()) != nullptr) {
    return this->ros2_update(pddl_action_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlActionDao::save_update(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  bool succ;
  if (this->get(pddl_action_dto->get_name()) == nullptr) {
    succ = this->save(pddl_action_dto);
  } else {
    succ = this->update(pddl_action_dto);
  }

  return succ;
}

bool Ros2PddlActionDao::delete_one(
    std::shared_ptr<kant::dto::PddlActionDto> pddl_action_dto) {
  return this->ros2_update(pddl_action_dto,
                           kant_msgs::msg::UpdateKnowledge::DELETE);
}

bool Ros2PddlActionDao::delete_all() {

  this->ros2_delete_all();
  return true;
}
