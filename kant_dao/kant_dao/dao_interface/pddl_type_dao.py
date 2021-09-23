
""" Pddl Type Dao Interface """

from abc import abstractmethod
from kant_dto import PddlTypeDto
from kant_dao.dao_interface import Dao


class PddlTypeDao(Dao):
    """ Pddl Type Dao Abstract Class """

    @abstractmethod
    def get(self, type_name: str) -> PddlTypeDto:
        """ get a PddlTypeDto with a given type name
            return None if there is no pddl with that type name

        Args:
            type_name (str): pddl type name

        Returns:
            PddlTypeDto: PddlTypeDto of the pddl type name
        """
