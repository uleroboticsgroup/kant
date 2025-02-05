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


"""Pddl Type Dto"""

from kant_dto.dto import Dto


class PddlTypeDto(Dto):
    """Pddl Type Dto Class"""

    def __init__(self, name: str):

        self.set_name(name)

        Dto.__init__(self)

    def get_name(self) -> str:
        """pddl type name getter

        Returns:
            str: pddl type name
        """

        return self._name

    def set_name(self, name: str):
        """pddl type name setter

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
