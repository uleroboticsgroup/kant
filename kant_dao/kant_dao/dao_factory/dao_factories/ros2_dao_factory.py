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


"""Ros2 Dao Factory"""

from rclpy.node import Node

from kant_dao.ros2_dao import (
    Ros2PddlTypeDao,
    Ros2PddlObjectDao,
    Ros2PddlPredicateDao,
    Ros2PddlPropositionDao,
    Ros2PddlActionDao,
)

from kant_dao.dao_factory.dao_factories.dao_factory import DaoFactory


class Ros2DaoFactory(DaoFactory):
    """Ros2 Dao Factory Class"""

    def __init__(self, node: Node):
        self.set_node(node)

    def get_node(self) -> Node:
        """node getter

        Returns:
            None: node
        """

        return self._node

    def set_node(self, node: Node):
        """node setter

        Args:
            node (None): Node
        """

        self._node = node

    def create_pddl_type_dao(self) -> Ros2PddlTypeDao:
        """create a kant dao type object

        Returns:
            Ros2PddlTypeDao: kant dao for pddl type
        """

        return Ros2PddlTypeDao(self._node)

    def create_pddl_predicate_dao(self) -> Ros2PddlPredicateDao:
        """create a kant dao predicate object

        Returns:
            Ros2PddlPredicateDao: kant dao for pddl predicate
        """

        return Ros2PddlPredicateDao(self._node)

    def create_pddl_action_dao(self) -> Ros2PddlActionDao:
        """create a kant dao action object

        Returns:
            Ros2PddlActionDao: kant dao for pddl action
        """

        return Ros2PddlActionDao(self._node)

    def create_pddl_object_dao(self) -> Ros2PddlObjectDao:
        """create a kant dao object object

        Returns:
            Ros2PddlObjectDao: kant dao for pddl object
        """

        return Ros2PddlObjectDao(self._node)

    def create_pddl_proposition_dao(self) -> Ros2PddlPropositionDao:
        """create a kant dao proposition object

        Returns:
            Ros2PddlPropositionDao: kant dao for pddl proposition
        """

        return Ros2PddlPropositionDao(self._node)
