
""" Pddl Predicate Dto """

from typing import List
from kant_dto.dto import Dto
from kant_dto.pddl_type_dto import PddlTypeDto


class PddlPredicateDto(Dto):
    """ Pddl Predicate Dto Class """

    def __init__(self, name: str, types: List[PddlTypeDto] = None):

        self.set_name(name)
        self.set_types(types)

        Dto.__init__(self)

    def get_name(self) -> str:
        """ pddl predicate name getter

        Returns:
            str: predicate name
        """

        return self._name

    def set_name(self, name: str):
        """ pddl predicate name setter

        Args:
            name (str): pddl predicate name
        """

        self._name = name

    def get_types(self) -> List[PddlTypeDto]:
        """ pddl types list getter

        Returns:
            List[PddlTypeDto]: list of pddl types
        """

        return self._types

    def set_types(self, types: List[PddlTypeDto]):
        """ pddl types list setter

        Args:
            types (List[PddlTypeDto]): list of pddl types
        """

        if types:
            self._types = types
        else:
            self._types = []

    def __str__(self):
        string = "(" + self._name

        if self._types:
            for i in range(len(self._types)):
                pddl_type = self._types[i]
                type_name = pddl_type.get_name()
                string += " ?" + type_name[0] + str(i) + " - " + type_name

        string += ")"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlPredicateDto):
            return self.get_name() == other.get_name()

        return False
