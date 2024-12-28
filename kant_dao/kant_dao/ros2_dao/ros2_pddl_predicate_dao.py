# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


""" Ros2 Pddl Predicate Dao """

from typing import List
from simple_node import Node

from kant_dao.dao_interface import PddlPredicateDao
from kant_dto import PddlPredicateDto

from kant_msgs.srv import (
    UpdatePddlPredicate,
    GetPddlPredicate
)
from kant_msgs.msg import UpdateKnowledge
from std_srvs.srv import Empty

from kant_knowledge_base.parser import (
    DtoMsgParser,
    MsgDtoParser
)


class Ros2PddlPredicateDao(PddlPredicateDao):
    """ Ros2 Pddl Predicate Dao Class """

    def __init__(self, node: Node):

        PddlPredicateDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlPredicate, "get_predicates")

        self._update_client = self.node.create_client(
            UpdatePddlPredicate, "update_predicate")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_predicates")

    def _ros2_get(self, predicate_name: str = "") -> List[PddlPredicateDto]:
        """ ros2_get method

        Args:
            predicate_name (str): predicate name

        Returns:
            List[PddlPredicateDto]: list of PddlPredicateDto
        """

        req = GetPddlPredicate.Request()

        req.predicate_name = predicate_name

        self._get_client.wait_for_service()
        result = self._get_client.call(req)

        pddl_predicate_dto_list = []

        for pddl_predicate_msg in result.pddl_predicates:
            pddl_predicate_dto = self.msg_dto_parser.predicate_msg_to_dto(
                pddl_predicate_msg)
            pddl_predicate_dto_list.append(pddl_predicate_dto)

        return pddl_predicate_dto_list

    def _ros2_update(self,
                     pddl_predicate_dto: PddlPredicateDto,
                     update_type: int) -> bool:
        """ ros2_update method

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

        Returns:
            bool: succeed
        """

        req = UpdatePddlPredicate.Request()

        req.pddl_predicate = self.dto_msg_parser.predicate_dto_to_msg(
            pddl_predicate_dto)
        req.update_konwledge.update_type = update_type

        self._update_client.wait_for_service()
        result = self._update_client.call(req)

        return result.success

    def _ros2_delete_all(self):
        """ ros2_delete_all method

        Returns:
            bool: succeed
        """

        req = Empty.Request()

        self._delete_all_client.wait_for_service()
        self._delete_all_client.call(req)

    def get(self, predicate_name: str) -> PddlPredicateDto:
        """ get a PddlPredicateDto with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto of the pddl predicate name
        """

        pddl_predicate_dto_list = self._ros2_get(predicate_name)

        if len(pddl_predicate_dto_list) == 1:
            return pddl_predicate_dto_list[0]

        return None

    def get_all(self) -> List[PddlPredicateDto]:
        """ get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of all PddlPredicateDto
        """

        pddl_predicate_dto_list = self._ros2_get()

        return pddl_predicate_dto_list

    def _save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save a PddlPredicateDto
            if the PddlPredicateDto is already saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_predicate_dto.get_name()):
            succ = self._ros2_update(
                pddl_predicate_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def _update(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ update a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_predicate_dto.get_name()):
            succ = self._ros2_update(
                pddl_predicate_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save or update a PddlPredicateDto
            if the PddlPredicateDto is not saved it will be saved, else it will be updated

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_predicate_dto.get_name()):
            succ = self._save(pddl_predicate_dto)
        else:
            succ = self._update(pddl_predicate_dto)

        return succ

    def delete(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ delete a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to delete

        Returns:
            bool: succeed
        """

        succ = self._ros2_update(
            pddl_predicate_dto, UpdateKnowledge.DELETE)
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl predicates

        Returns:
            bool: succeed
        """

        self._ros2_delete_all()

        return True
