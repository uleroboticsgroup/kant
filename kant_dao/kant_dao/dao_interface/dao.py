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


""" Dao Interface """

from abc import ABC, abstractmethod
from typing import List
from kant_dto import Dto


class Dao(ABC):
    """ Dao Abstract Class """

    @abstractmethod
    def get_all(self) -> List[Dto]:
        """ get all Dto

        Returns:
            List[Dto]: list of all Dto
        """

    @abstractmethod
    def _save(self, pdd_dto: Dto) -> bool:
        """ save a Dto
            if the Dto is already saved return False, else return True

        Args:
            pdd_dto (Dto): Dto to save

        Returns:
            bool: succeed
        """

    @abstractmethod
    def _update(self, pdd_dto: Dto) -> bool:
        """ update a Dto
            if the Dto is not saved return False, else return True

        Args:
            pdd_dto (Dto): Dto to update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def save(self, pdd_dto: Dto) -> bool:
        """ save or update a Dto
            if the Dto is not saved it will be saved, else it will be updated

        Args:
            pdd_dto (Dto): Dto to save or update

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete(self, pdd_dto: Dto) -> bool:
        """ delete a Dto
            if the Dto is not saved return False, else return True

        Args:
            pdd_dto (Dto): Dto to delete

        Returns:
            bool: succeed
        """

    @abstractmethod
    def delete_all(self) -> bool:
        """ delete all pddl dtos

        Returns:
            bool: succeed
        """
