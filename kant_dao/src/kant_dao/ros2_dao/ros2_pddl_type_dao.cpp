// Copyright (C) 2023 Miguel Ángel González Santamarta

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

#include "kant_dao/ros2_dao/ros2_pddl_type_dao.hpp"

using namespace kant::dao::ros2_dao;

Ros2PddlTypeDao::Ros2PddlTypeDao(simple_node::Node *node) {

  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();

  // srv clients
  this->get_client =
      node->create_client<kant_msgs::srv::GetPddlType>("get_types");

  this->update_client =
      node->create_client<kant_msgs::srv::UpdatePddlType>("update_type");

  this->delete_all_client =
      node->create_client<std_srvs::srv::Empty>("delete_all_types");
}

std::vector<std::shared_ptr<kant::dto::PddlTypeDto>>
Ros2PddlTypeDao::ros2_get(std::string type_name) {

  auto request = std::make_shared<kant_msgs::srv::GetPddlType::Request>();

  request->type_name = type_name;

  this->get_client->wait_for_service();
  auto future = this->get_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  std::vector<std::shared_ptr<kant::dto::PddlTypeDto>> pddl_type_dto_list;

  for (const auto &pddl_type_msg : result->pddl_types) {
    auto pddl_type_dto = this->msg_dto_parser->type_msg_to_dto(pddl_type_msg);
    pddl_type_dto_list.push_back(pddl_type_dto);
  }

  return pddl_type_dto_list;
}

bool Ros2PddlTypeDao::ros2_update(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto, int update_type) {

  auto request = std::make_shared<kant_msgs::srv::UpdatePddlType::Request>();

  request->pddl_type = this->dto_msg_parser->type_dto_to_msg(pddl_type_dto);
  request->update_konwledge.update_type = update_type;

  this->update_client->wait_for_service();
  auto future = this->update_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  return result->success;
}

void Ros2PddlTypeDao::ros2_delete_all() {

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  this->delete_all_client->wait_for_service();
  auto future = this->delete_all_client->async_send_request(request);
  future.wait();
}

std::shared_ptr<kant::dto::PddlTypeDto>
Ros2PddlTypeDao::get(std::string type_name) {

  auto pddl_type_dto_list = this->ros2_get(type_name);

  if (pddl_type_dto_list.size() == 1) {
    return pddl_type_dto_list.at(0);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlTypeDto>>
Ros2PddlTypeDao::get_all() {

  auto pddl_type_dto_list = this->ros2_get("");
  return pddl_type_dto_list;
}

bool Ros2PddlTypeDao::save(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {

  if (this->get(pddl_type_dto->get_name()) == nullptr) {
    return this->ros2_update(pddl_type_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlTypeDao::update(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {
  if (this->get(pddl_type_dto->get_name()) != nullptr) {
    return this->ros2_update(pddl_type_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlTypeDao::save_update(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {
  bool succ;
  if (this->get(pddl_type_dto->get_name()) == nullptr) {
    succ = this->save(pddl_type_dto);
  } else {
    succ = this->update(pddl_type_dto);
  }

  return succ;
}

bool Ros2PddlTypeDao::delete_one(
    std::shared_ptr<kant::dto::PddlTypeDto> pddl_type_dto) {
  return this->ros2_update(pddl_type_dto,
                           kant_msgs::msg::UpdateKnowledge::DELETE);
}

bool Ros2PddlTypeDao::delete_all() {

  this->ros2_delete_all();
  return true;
}
