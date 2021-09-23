
""" Pddl Object Dto """

from kant_dto.dto import Dto
from kant_dto.pddl_type_dto import PddlTypeDto


class PddlObjectDto(Dto):
    """ Pddl Object Dto Class """

    def __init__(self, type: PddlTypeDto, name: str):

        self.set_type(type)
        self.set_name(name)

        Dto.__init__(self)

    def get_type(self) -> PddlTypeDto:
        """ pddl type getter

        Returns:
            PddlTypeDto: pddl type
        """

        return self._type

    def set_type(self, type: PddlTypeDto):
        """ pddl type setter

        Args:
            type (PddlTypeDto): pddl type
        """

        self._type = type

    def get_name(self) -> str:
        """ pddl object name getter

        Returns:
            str: pddl object name
        """

        return self._name

    def set_name(self, name: str):
        """ pddl object name setter

        Args:
            name (str): pddl object name
        """

        self._name = name

    def __str__(self):
        return self._name + " - " + self._type.get_name()

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlObjectDto):
            return self.get_name() == other.get_name()

        return False
