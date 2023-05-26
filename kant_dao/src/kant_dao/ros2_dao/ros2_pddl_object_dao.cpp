
#include "kant_msgs/msg/update_knowledge.hpp"

#include "kant_dao/ros2_dao/ros2_pddl_object_dao.hpp"

using namespace kant::dao::ros2_dao;

Ros2PddlObjectDao::Ros2PddlObjectDao(simple_node::Node *node) {

  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();

  // srv clients
  this->get_client =
      node->create_client<kant_msgs::srv::GetPddlObject>("get_objects");

  this->update_client =
      node->create_client<kant_msgs::srv::UpdatePddlObject>(
          "update_object");

  this->delete_all_client =
      node->create_client<std_srvs::srv::Empty>("delete_all_objects");
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
Ros2PddlObjectDao::ros2_get(std::string object_name) {

  auto request =
      std::make_shared<kant_msgs::srv::GetPddlObject::Request>();

  request->object_name = object_name;

  this->get_client->wait_for_service();
  auto future = this->get_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>> pddl_object_dto_list;

  for (const auto &pddl_object_msg : result->pddl_objects) {
    auto pddl_object_dto =
        this->msg_dto_parser->object_msg_to_dto(pddl_object_msg);
    pddl_object_dto_list.push_back(pddl_object_dto);
  }

  return pddl_object_dto_list;
}

bool Ros2PddlObjectDao::ros2_update(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto,
    int update_type) {

  auto request =
      std::make_shared<kant_msgs::srv::UpdatePddlObject::Request>();

  request->pddl_object =
      this->dto_msg_parser->object_dto_to_msg(pddl_object_dto);
  request->update_konwledge.update_type = update_type;

  this->update_client->wait_for_service();
  auto future = this->update_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  return result->success;
}

void Ros2PddlObjectDao::ros2_delete_all() {

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  this->delete_all_client->wait_for_service();
  auto future = this->delete_all_client->async_send_request(request);
  future.wait();
}

std::shared_ptr<kant::dto::PddlObjectDto>
Ros2PddlObjectDao::get(std::string object_name) {

  auto pddl_object_dto_list = this->ros2_get(object_name);

  if (pddl_object_dto_list.size() == 1) {
    return pddl_object_dto_list.at(0);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
Ros2PddlObjectDao::get_all() {

  auto pddl_object_dto_list = this->ros2_get("");
  return pddl_object_dto_list;
}

bool Ros2PddlObjectDao::save(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {

  if (this->get(pddl_object_dto->get_name()) == nullptr) {
    return this->ros2_update(pddl_object_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlObjectDao::update(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {
  if (this->get(pddl_object_dto->get_name()) != nullptr) {
    return this->ros2_update(pddl_object_dto,
                             kant_msgs::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlObjectDao::save_update(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {
  bool succ;
  if (this->get(pddl_object_dto->get_name()) == nullptr) {
    succ = this->save(pddl_object_dto);
  } else {
    succ = this->update(pddl_object_dto);
  }

  return succ;
}

bool Ros2PddlObjectDao::delete_one(
    std::shared_ptr<kant::dto::PddlObjectDto> pddl_object_dto) {
  return this->ros2_update(pddl_object_dto,
                           kant_msgs::msg::UpdateKnowledge::DELETE);
}

bool Ros2PddlObjectDao::delete_all() {

  this->ros2_delete_all();
  return true;
}
