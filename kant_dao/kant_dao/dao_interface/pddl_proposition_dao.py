
""" Pddl Proposition Dao Interface """

from abc import abstractmethod
from typing import List
from kant_dto import PddlPropositionDto
from kant_dao.dao_interface import Dao


class PddlPropositionDao(Dao):
    """ Pddl Proposition Dao Abstract Class """

    @abstractmethod
    def get_by_predicate(self, predicate_name: str) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto with a given pddl predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

    @abstractmethod
    def get_goals(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto that are goals

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

    @abstractmethod
    def get_no_goals(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto that are not goals

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

    @abstractmethod
    def get_all(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """
