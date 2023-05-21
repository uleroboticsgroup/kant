
#include "kant_interfaces/msg/update_knowledge.hpp"

#include "kant_dao/ros2_dao/ros2_pddl_predicate_dao.hpp"

using namespace kant::dao::ros2_dao;

Ros2PddlPredicateDao::Ros2PddlPredicateDao(simple_node::Node *node) {

  this->dto_msg_parser =
      std::make_unique<kant::knowledge_base::parser::DtoMsgParser>();
  this->msg_dto_parser =
      std::make_unique<kant::knowledge_base::parser::MsgDtoParser>();

  // srv clients
  this->get_client =
      node->create_client<kant_interfaces::srv::GetPddlPredicate>(
          "get_predicates");

  this->update_client =
      node->create_client<kant_interfaces::srv::UpdatePddlPredicate>(
          "update_predicate");

  this->delete_all_client =
      node->create_client<std_srvs::srv::Empty>("delete_all_predicates");
}

std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
Ros2PddlPredicateDao::ros2_get(std::string predicate_name) {

  auto request =
      std::make_shared<kant_interfaces::srv::GetPddlPredicate::Request>();

  request->predicate_name = predicate_name;

  this->get_client->wait_for_service();
  auto future = this->get_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
      pddl_predicate_dto_list;

  for (const auto &pddl_predicate_msg : result->pddl_predicates) {
    auto pddl_predicate_dto =
        this->msg_dto_parser->predicate_msg_to_dto(pddl_predicate_msg);
    pddl_predicate_dto_list.push_back(pddl_predicate_dto);
  }

  return pddl_predicate_dto_list;
}

bool Ros2PddlPredicateDao::ros2_update(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto,
    int update_type) {

  auto request =
      std::make_shared<kant_interfaces::srv::UpdatePddlPredicate::Request>();

  request->pddl_predicate =
      this->dto_msg_parser->predicate_dto_to_msg(pddl_predicate_dto);
  request->update_konwledge.update_type = update_type;

  this->update_client->wait_for_service();
  auto future = this->update_client->async_send_request(request);
  future.wait();
  auto result = future.get();

  return result->success;
}

void Ros2PddlPredicateDao::ros2_delete_all() {

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  this->delete_all_client->wait_for_service();
  auto future = this->delete_all_client->async_send_request(request);
  future.wait();
}

std::shared_ptr<kant::dto::PddlPredicateDto>
Ros2PddlPredicateDao::get(std::string predicate_name) {

  auto pddl_predicate_dto_list = this->ros2_get(predicate_name);

  if (pddl_predicate_dto_list.size() == 1) {
    return pddl_predicate_dto_list.at(0);
  }

  return nullptr;
}

std::vector<std::shared_ptr<kant::dto::PddlPredicateDto>>
Ros2PddlPredicateDao::get_all() {

  auto pddl_predicate_dto_list = this->ros2_get("");
  return pddl_predicate_dto_list;
}

bool Ros2PddlPredicateDao::save(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {

  if (this->get(pddl_predicate_dto->get_name()) == nullptr) {
    return this->ros2_update(pddl_predicate_dto,
                             kant_interfaces::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlPredicateDao::update(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {
  if (this->get(pddl_predicate_dto->get_name()) != nullptr) {
    return this->ros2_update(pddl_predicate_dto,
                             kant_interfaces::msg::UpdateKnowledge::SAVE);
  }

  return false;
}

bool Ros2PddlPredicateDao::save_update(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {
  bool succ;
  if (this->get(pddl_predicate_dto->get_name()) == nullptr) {
    succ = this->save(pddl_predicate_dto);
  } else {
    succ = this->update(pddl_predicate_dto);
  }

  return succ;
}

bool Ros2PddlPredicateDao::delete_one(
    std::shared_ptr<kant::dto::PddlPredicateDto> pddl_predicate_dto) {
  return this->ros2_update(pddl_predicate_dto,
                           kant_interfaces::msg::UpdateKnowledge::DELETE);
}

bool Ros2PddlPredicateDao::delete_all() {

  this->ros2_delete_all();
  return true;
}
