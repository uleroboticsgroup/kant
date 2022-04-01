
#include "kant_interfaces/msg/update_knowledge.hpp"

#include "kant_dao/ros2_dao/ros2_pddl_proposition_dao.hpp"

using namespace kant::dao::ros2_dao;

Ros2PddlPropositionDao::Ros2PddlPropositionDao(simple_node::Node *node) {

  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();

  // srv clients
  this->get_client =
      node->create_client<kant_interfaces::srv::GetPddlProposition>(
          "get_propositions");

  this->update_client =
      node->create_client<kant_interfaces::srv::UpdatePddlProposition>(
          "update_proposition");

  this->delete_all_client =
      node->create_client<std_srvs::srv::Empty>("delete_all_propositions");
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
Ros2PddlPropositionDao::ros2_get(int get_type, std::string predicate_name) {

  auto request =
      std::make_shared<kant_interfaces::srv::GetPddlProposition::Request>();

  request->predicate_name = predicate_name;
  request->get_type = get_type;

  this->get_client->wait_for_service();
  auto future = this->get_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
      pddl_proposition_dto_list;

  for (const auto &pddl_proposition_msg : result->pddl_propositions) {
    auto pddl_proposition_dto =
        this->msg_dto_parser->proposition_msg_to_dto(pddl_proposition_msg);
    pddl_proposition_dto_list.push_back(pddl_proposition_dto);
  }

  return pddl_proposition_dto_list;
}

bool Ros2PddlPropositionDao::ros2_update(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto,
    int update_type) {

  auto request =
      std::make_shared<kant_interfaces::srv::UpdatePddlProposition::Request>();

  request->pddl_proposition =
      this->dto_msg_parser->proposition_dto_to_msg(pddl_proposition_dto);
  request->update_konwledge.update_type = update_type;

  this->update_client->wait_for_service();
  auto future = this->update_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  return result->success;
}

void Ros2PddlPropositionDao::ros2_delete_all() {

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  this->delete_all_client->wait_for_service();
  auto future = this->delete_all_client->async_send_request(request);
  future.wait();
}

bool Ros2PddlPropositionDao::ros2_exists(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  std::string predicate_name =
      pddl_proposition_dto->get_predicate()->get_name();

  for (const auto &dto : this->get_by_predicate(predicate_name)) {
    if (dto->equals(pddl_proposition_dto)) {
      return true;
    }
  }

  return false;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
Ros2PddlPropositionDao::get_by_predicate(std::string predicate_name) {

  auto pddl_proposition_dto_list = this->ros2_get(
      kant_interfaces::srv::GetPddlProposition::Request::BY_PREDICATE,
      predicate_name);

  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
Ros2PddlPropositionDao::get_goals() {

  auto pddl_proposition_dto_list = this->ros2_get(
      kant_interfaces::srv::GetPddlProposition::Request::GOALS, "");

  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
Ros2PddlPropositionDao::get_no_goals() {

  auto pddl_proposition_dto_list = this->ros2_get(
      kant_interfaces::srv::GetPddlProposition::Request::NO_GOALS, "");

  return pddl_proposition_dto_list;
}

std::vector<std::shared_ptr<kant::dto::PddlPropositionDto>>
Ros2PddlPropositionDao::get_all() {

  auto pddl_proposition_dto_list = this->ros2_get(
      kant_interfaces::srv::GetPddlProposition::Request::ALL, "");

  return pddl_proposition_dto_list;
}

bool Ros2PddlPropositionDao::save(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (!this->ros2_exists(pddl_proposition_dto)) {
    bool succ = this->ros2_update(pddl_proposition_dto,
                                  kant_interfaces::msg::UpdateKnowledge::SAVE);
    return succ;
  }

  return false;
}

bool Ros2PddlPropositionDao::update(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  if (this->ros2_exists(pddl_proposition_dto)) {
    bool succ = this->ros2_update(pddl_proposition_dto,
                                  kant_interfaces::msg::UpdateKnowledge::SAVE);
    return succ;
  }

  return false;
}

bool Ros2PddlPropositionDao::save_update(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {

  bool succ;
  if (!this->ros2_exists(pddl_proposition_dto)) {
    succ = this->save(pddl_proposition_dto);
  } else {
    succ = this->update(pddl_proposition_dto);
  }

  return succ;
}

bool Ros2PddlPropositionDao::delete_one(
    std::shared_ptr<kant::dto::PddlPropositionDto> pddl_proposition_dto) {
  bool succ = this->ros2_update(pddl_proposition_dto,
                                kant_interfaces::msg::UpdateKnowledge::DELETE);
  return succ;
}

bool Ros2PddlPropositionDao::delete_all() {

  this->ros2_delete_all();
  return true;
}
