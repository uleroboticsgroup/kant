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
