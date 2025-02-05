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


"""Mongo Dao Factory"""

from mongoengine import disconnect, connect

from kant_dao.mongo_dao import (
    MongoPddlTypeDao,
    MongoPddlObjectDao,
    MongoPddlPredicateDao,
    MongoPddlPropositionDao,
    MongoPddlActionDao,
)

from kant_dao.dao_factory.dao_factories.dao_factory import DaoFactory


class MongoDaoFactory(DaoFactory):
    """Mongo Dao Factory Class"""

    def __init__(self, uri: str = "mongodb://localhost:27017/kant"):
        self.set_uri(uri)
        self.connect()

    def connect(self):
        """connect to current uri"""

        disconnect()
        connect(host=self._uri)

    def get_uri(self) -> str:
        """uri getter

        Returns:
            str: uri str
        """

        return self._uri

    def set_uri(self, uri: str):
        """uri setter

        Args:
            uri (str): uri str
        """

        self._uri = uri

    def create_pddl_type_dao(self) -> MongoPddlTypeDao:
        """create a mongo pddl dao type object

        Returns:
            MongoPddlTypeDao: mongoengine dao for pddl type
        """

        return MongoPddlTypeDao(uri=self._uri, connect=False)

    def create_pddl_predicate_dao(self) -> MongoPddlPredicateDao:
        """create a mongo pddl dao predicate object

        Returns:
            MongoPddlPredicateDao: mongoengine dao for pddl predicate
        """

        return MongoPddlPredicateDao(uri=self._uri, connect=False)

    def create_pddl_action_dao(self) -> MongoPddlActionDao:
        """create a mongo pddl dao action object

        Returns:
            MongoPddlActionDao: mongoengine dao for pddl action
        """

        return MongoPddlActionDao(uri=self._uri, connect=False)

    def create_pddl_object_dao(self) -> MongoPddlObjectDao:
        """create a mongo pddl dao object object

        Args:
            uri (str, optional): Mongo uri. Defaults to None.

        Returns:
            MongoPddlObjectDao: mongoengine dao for pddl object
        """

        return MongoPddlObjectDao(uri=self._uri, connect=False)

    def create_pddl_proposition_dao(self) -> MongoPddlPropositionDao:
        """create a mongo pddl dao proposition object

        Returns:
            MongoPddlPropositionDao: mongoengine dao for pddl proposition
        """

        return MongoPddlPropositionDao(uri=self._uri, connect=False)
