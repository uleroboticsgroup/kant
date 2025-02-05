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


"""Dao Factory Interface"""

from abc import ABC, abstractmethod
from kant_dao.dao_interface import (
    PddlTypeDao,
    PddlObjectDao,
    PddlPredicateDao,
    PddlPropositionDao,
    PddlActionDao,
)


class DaoFactory(ABC):
    """Dao Factory Abstract Class"""

    @abstractmethod
    def create_pddl_type_dao(self) -> PddlTypeDao:
        """create a pddl dao type object

        Returns:
            PddlTypeDao: dao for pddl type
        """

    @abstractmethod
    def create_pddl_predicate_dao(self) -> PddlPredicateDao:
        """create a pddl dao predicate object

        Returns:
            PddlPredicateDao: dao for pddl predicate
        """

    @abstractmethod
    def create_pddl_action_dao(self) -> PddlActionDao:
        """create a pddl dao action object

        Returns:
            PddlActionDao: dao for pddl action
        """

    @abstractmethod
    def create_pddl_object_dao(self) -> PddlObjectDao:
        """create a pddl dao object object

        Returns:
            PddlObjectDao: dao for pddl object
        """

    @abstractmethod
    def create_pddl_proposition_dao(self) -> PddlPropositionDao:
        """create a pddl dao type object

        Returns:
            PddlPropositionDao: dao for pddl proposition
        """
