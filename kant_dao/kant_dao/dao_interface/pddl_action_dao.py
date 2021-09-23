
""" Pddl Action Dao Interface """

from abc import abstractmethod
from kant_dto import PddlActionDto
from kant_dao.dao_interface import Dao


class PddlActionDao(Dao):
    """ Pddl Action Dao Abstract Class """

    @abstractmethod
    def get(self, action_name: str) -> PddlActionDto:
        """ get a PddlActionDto with a given action name
            return None if there is no pddl with that action name

        Args:
            action_name (str): pddl action name

        Returns:
            PddlActionDto: PddlActionDto of the pddl action name
        """
