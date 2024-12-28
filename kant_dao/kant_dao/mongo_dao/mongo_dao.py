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


""" Mongo Dao Interface """

from abc import ABC, abstractmethod
from mongoengine import Document, disconnect, connect
from kant_dto import Dto


class MongoDao(ABC):
    """ Mongo Dao Abstract Class """

    def __init__(self, uri: str = "mongodb://localhost:27017/kant"):
        self.set_uri(uri)

    def connect(self):
        """ connect to current uri
        """

        disconnect()
        connect(host=self._uri)

    def get_uri(self) -> str:
        """ uri getter

        Returns:
            str: Mongo uri
        """

        return self._uri

    def set_uri(self, uri: str):
        """ uri setter

        Args:
            uri (str): Mongo uri
        """

        self._uri = uri

    @abstractmethod
    def _get_model(self, dto: Dto) -> Document:
        """ get the Mongoengine document corresponding to a give Dto

        Args:
            dto (Dto): Dto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _model_to_dto(self, pddl_mongoengine: Document) -> Dto:
        """ convert a Mongoengine document into a Dto

        Args:
            pddl_mongoengine (Document): Mongoengine document

        Returns:
            Dto: Dto
        """

    @abstractmethod
    def _dto_to_model(self, dto: Dto) -> Document:
        """ convert a Dto into a Mongoengine document

        Args:
            dto (Dto): Dto

        Returns:
            Document: Mongoengine document
        """

    @abstractmethod
    def _exist_in_mongo(self, dto: Dto) -> bool:
        """ check if Dto exists

        Args:
            dto (Dto): Dto

        Returns:
            bool: Dto exists?
        """
