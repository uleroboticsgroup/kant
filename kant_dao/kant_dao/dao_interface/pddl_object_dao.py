
""" Pddl Object Dao Interface """

from abc import abstractmethod
from kant_dto import PddlObjectDto
from kant_dao.dao_interface import Dao


class PddlObjectDao(Dao):
    """ Pddl Object Dao Interface Abstract Class"""

    @abstractmethod
    def get(self, object_name: str) -> PddlObjectDto:
        """ get a PddlObjectDto with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlObjectDto: PddlObjectDto of the pddl object name
        """
