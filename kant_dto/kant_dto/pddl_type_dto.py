
""" Pddl Type Dto """

from kant_dto.dto import Dto


class PddlTypeDto(Dto):
    """ Pddl Type Dto Class """

    def __init__(self, name: str):

        self.set_name(name)

        Dto.__init__(self)

    def get_name(self) -> str:
        """ pddl type name getter

        Returns:
            str: pddl type name
        """

        return self._name

    def set_name(self, name: str):
        """ pddl type name setter

        Args:
            name (str): pddl type name
        """

        self._name = name

    def __str__(self):
        return self._name

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlTypeDto):
            return self.get_name() == other.get_name()

        return False
