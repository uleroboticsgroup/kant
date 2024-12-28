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


""" Ros2 Pddl Action Dao """

from typing import List
from simple_node import Node

from kant_dao.dao_interface import PddlActionDao
from kant_dto import PddlActionDto

from kant_msgs.srv import (
    UpdatePddlAction,
    GetPddlAction
)
from kant_msgs.msg import UpdateKnowledge
from std_srvs.srv import Empty

from kant_knowledge_base.parser import (
    DtoMsgParser,
    MsgDtoParser
)


class Ros2PddlActionDao(PddlActionDao):
    """ Ros2 Pddl Action Dao Class """

    def __init__(self, node: Node):

        PddlActionDao.__init__(self)

        self.node = node

        # parsers
        self.dto_msg_parser = DtoMsgParser()
        self.msg_dto_parser = MsgDtoParser()

        # srv clients
        self._get_client = self.node.create_client(
            GetPddlAction, "get_actions")

        self._update_client = self.node.create_client(
            UpdatePddlAction, "update_action")

        self._delete_all_client = self.node.create_client(
            Empty, "delete_all_actions")

    def _ros2_get(self, action_name: str = "") -> List[PddlActionDto]:
        """ ros2_get method

        Args:
            action_name (str): action name

        Returns:
            List[PddlActionDto]: list of PddlActionDto
        """

        req = GetPddlAction.Request()

        req.action_name = action_name

        self._get_client.wait_for_service()
        result = self._get_client.call(req)

        pddl_action_dto_list = []

        for pddl_action_msg in result.pddl_actions:
            pddl_action_dto = self.msg_dto_parser.action_msg_to_dto(
                pddl_action_msg)
            pddl_action_dto_list.append(pddl_action_dto)

        return pddl_action_dto_list

    def _ros2_update(self,
                     pddl_action_dto: PddlActionDto,
                     update_type: int) -> bool:
        """ ros2_delete_all method

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to update

        Returns:
            bool: succeed
        """

        req = UpdatePddlAction.Request()

        req.pddl_action = self.dto_msg_parser.action_dto_to_msg(
            pddl_action_dto)
        req.update_konwledge.update_type = update_type

        self._update_client.wait_for_service()
        result = self._update_client.call(req)

        return result.success

    def _ros2_delete_all(self):
        """ asyn delete_all method

        Returns:
            bool: succeed
        """

        req = Empty.Request()

        self._delete_all_client.wait_for_service()
        self._delete_all_client.call(req)

    def get(self, action_name: str) -> PddlActionDto:
        """ get a PddlActionDto with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlActionDto: PddlActionDto of the pddl action name
        """

        pddl_action_dto_list = self._ros2_get(action_name)

        if len(pddl_action_dto_list) == 1:
            return pddl_action_dto_list[0]

        return None

    def get_all(self) -> List[PddlActionDto]:
        """ get all PddlActionDto

        Returns:
            List[PddlActionDto]: list of all PddlActionDto
        """

        pddl_action_dto_list = self._ros2_get()

        return pddl_action_dto_list

    def _save(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save a PddlActionDto
            if the PddlActionDto is already saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save

        Returns:
            bool: succeed
        """

        if not self.get(pddl_action_dto.get_name()):
            succ = self._ros2_update(
                pddl_action_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def _update(self, pddl_action_dto: PddlActionDto) -> bool:
        """ update a PddlActionDto
            if the PddlActionDto is not saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to update

        Returns:
            bool: succeed
        """

        if self.get(pddl_action_dto.get_name()):
            succ = self._ros2_update(
                pddl_action_dto, UpdateKnowledge.SAVE)
            return succ

        return False

    def save(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save or update a PddlActionDto
            if the PddlActionDto is not saved it will be saved, else it will be updated

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to save or update

        Returns:
            bool: succeed
        """

        if not self.get(pddl_action_dto.get_name()):
            succ = self._save(pddl_action_dto)
        else:
            succ = self._update(pddl_action_dto)

        return succ

    def delete(self, pddl_action_dto: PddlActionDto) -> bool:
        """ delete a PddlActionDto
            if the PddlActionDto is not saved return False, else return True

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto to delete

        Returns:
            bool: succeed
        """

        succ = self._ros2_update(
            pddl_action_dto, UpdateKnowledge.DELETE)
        return succ

    def delete_all(self) -> bool:
        """ delete all pddl actions

        Returns:
            bool: succeed
        """

        self._ros2_delete_all()

        return True
