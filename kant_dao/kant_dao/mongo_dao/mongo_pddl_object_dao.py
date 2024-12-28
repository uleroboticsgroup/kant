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


""" Mongo Pddl Object Dao """

from typing import List

from kant_dao.dao_interface import PddlObjectDao
from kant_dao.mongo_dao import (
    MongoDao,
    MongoPddlTypeDao
)

from kant_dao.mongo_dao.mongo_models import PddlObjectModel


from kant_dto import (
    PddlObjectDto,
    PddlTypeDto
)


class MongoPddlObjectDao(PddlObjectDao, MongoDao):
    """ Mongo Pddl Object Dao Class """

    def __init__(self, uri: str = None, connect: bool = True):

        PddlObjectDao.__init__(self)
        MongoDao.__init__(self, uri)

        if connect:
            self.connect()

        self._me_pddl_type_dao = MongoPddlTypeDao(uri, connect=False)

    def _model_to_dto(self, pddl_object_model: PddlObjectModel) -> PddlObjectDto:
        """ convert a Mongoengine pddl object document into a PddlObjectDto

        Args:
            pddl_object_model (PddlObjectModel): Mongoengine pddl object document

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        pddl_type_dto = PddlTypeDto(
            pddl_object_model.type.name)

        pddl_object_dto = PddlObjectDto(pddl_type_dto,
                                        pddl_object_model.name)

        return pddl_object_dto

    def _dto_to_model(self, pddl_object_dto: PddlObjectDto) -> PddlObjectModel:
        """ convert a PddlObjectDto into a Mongoengine pddl object document

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            Document: Mongoengine pddl object document
        """

        pddl_type_model = self._me_pddl_type_dao._dto_to_model(
            pddl_object_dto.get_type())

        pddl_object_model = PddlObjectModel()

        pddl_object_model.name = pddl_object_dto.get_name()

        pddl_object_model.type = pddl_type_model

        return pddl_object_model

    def _exist_in_mongo(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ check if PddlObjectDto exists

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: PddlObjectDto exists?
        """

        if self._get_model(pddl_object_dto):
            return True
        return False

    def _get_model(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ get the Mongoengine pddl object document corresponding to a give PddlObjectDto

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            Document: Mongoengine pddl object document
        """

        pddl_object_model = PddlObjectModel.objects(
            name=pddl_object_dto.get_name())

        if not pddl_object_model:
            return None

        return pddl_object_model[0]

    def get(self, object_name: str) -> PddlObjectDto:
        """ get a PddlObjectDto with a given object name
            return None if there is no pddl with that object name

        Args:
            object_name (str): pddl object name

        Returns:
            PddlObjectDto: PddlObjectDto of the pddl object name
        """

        pddl_object_model = PddlObjectModel.objects(
            name=object_name)

        # check if object exists
        if pddl_object_model:
            pddl_object_model = pddl_object_model[0]
            pddl_object_dto = self._model_to_dto(
                pddl_object_model)
            return pddl_object_dto

        return None

    def get_all(self) -> List[PddlObjectDto]:
        """ get all PddlObjectDto

        Returns:
            List[PddlObjectDto]: list of all PddlObjectDto
        """

        pddl_object_model = PddlObjectModel.objects.order_by("name")
        pddl_object_dto_list = []

        for ele in pddl_object_model:
            pddl_object_dto = self._model_to_dto(ele)
            pddl_object_dto_list.append(pddl_object_dto)

        return pddl_object_dto_list

    def _save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save a PddlObjectDto
            if the PddlObjectDto is already saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_object_dto):
            return False

        pddl_object_model = self._dto_to_model(
            pddl_object_dto)

        if pddl_object_model:
            pddl_object_model.save(cascade=True)
            return True

        return False

    def _update(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ update a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to update

        Returns:
            bool: succeed
        """

        pddl_object_model = self._get_model(pddl_object_dto)

        # check if object exists
        if pddl_object_model:
            new_pddl_object_model = self._dto_to_model(
                pddl_object_dto)

            pddl_object_model.name = new_pddl_object_model.name
            pddl_object_model.type = new_pddl_object_model.type
            pddl_object_model.save()

            return True

        return False

    def save(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save or update a PddlObjectDto
            if the PddlObjectDto is not saved it will be saved, else it will be updated

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_object_dto):
            return self._update(pddl_object_dto)

        return self._save(pddl_object_dto)

    def delete(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ delete a PddlObjectDto
            if the PddlObjectDto is not saved return False, else return True

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto to delete

        Returns:
            bool: succeed
        """

        pddl_object_model = self._get_model(pddl_object_dto)

        # check if object exists
        if pddl_object_model:
            pddl_object_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """ delete all pddl objects

        Returns:
            bool: succeed
        """

        pddl_object_dto_list = self.get_all()

        for ele in pddl_object_dto_list:
            self.delete(ele)

        return True
