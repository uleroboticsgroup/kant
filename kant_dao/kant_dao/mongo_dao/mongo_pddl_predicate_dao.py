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


"""Mongo Pddl Predicate Dao"""

from typing import List

from kant_dao.dao_interface import PddlPredicateDao
from kant_dao.mongo_dao import MongoDao, MongoPddlTypeDao

from kant_dao.mongo_dao.mongo_models import PddlPredicateModel

from kant_dto import PddlPredicateDto, PddlTypeDto


class MongoPddlPredicateDao(PddlPredicateDao, MongoDao):
    """Mongo Pddl Predicate Dao Class"""

    def __init__(self, uri: str = None, connect: bool = True):

        PddlPredicateDao.__init__(self)
        MongoDao.__init__(self, uri)

        if connect:
            self.connect()

        self._me_pddl_type_dao = MongoPddlTypeDao(uri, connect=False)

    def _model_to_dto(self, pddl_predicate_model: PddlPredicateModel) -> PddlPredicateDto:
        """convert a Mongoengine pddl predicate document into a PddlPredicateDto

        Args:
            pddl_predicate_model (PddlPredicateModel): Mongoengine pddl predicate document

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        pddl_type_dto_list = []

        for pddl_type_model in pddl_predicate_model.types:
            pddl_type_dto = PddlTypeDto(pddl_type_model.name)
            pddl_type_dto_list.append(pddl_type_dto)

        pddl_predicate_dto = PddlPredicateDto(
            pddl_predicate_model.name, pddl_type_dto_list
        )

        return pddl_predicate_dto

    def _dto_to_model(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicateModel:
        """convert a PddlPredicateDto into a Mongoengine pddl predicate document

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            Document: Mongoengine pddl predicate document
        """

        pddl_predicate_model = PddlPredicateModel()

        pddl_predicate_model.name = pddl_predicate_dto.get_name()

        for pddl_type_dto in pddl_predicate_dto.get_types():

            pddl_type_model = self._me_pddl_type_dao._dto_to_model(pddl_type_dto)

            pddl_predicate_model.types.append(pddl_type_model)

        return pddl_predicate_model

    def _exist_in_mongo(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """check if PddlPredicateDto exists

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: PddlPredicateDto exists?
        """

        if self._get_model(pddl_predicate_dto):
            return True
        return False

    def _get_model(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicateModel:
        """get the Mongoengine pddl predicate document corresponding to a give PddlPredicateDto

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            Document: Mongoengine pddl predicate document
        """

        pddl_predicate_model = PddlPredicateModel.objects(
            name=pddl_predicate_dto.get_name()
        )

        if not pddl_predicate_model:
            return None

        return pddl_predicate_model[0]

    def get(self, predicate_name: str) -> PddlPredicateDto:
        """get a PddlPredicateDto with a given predicate name
            return None if there is no pddl with that predicate name

        Args:
            predicate_name (str): pddl predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto of the pddl predicate name
        """

        pddl_predicate_model = PddlPredicateModel.objects(name=predicate_name)

        # check if predicate exist
        if pddl_predicate_model:
            pddl_predicate_model = pddl_predicate_model[0]
            pddl_predicate_dto = self._model_to_dto(pddl_predicate_model)
            return pddl_predicate_dto

        return None

    def get_all(self) -> List[PddlPredicateDto]:
        """get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of all PddlPredicateDto
        """
        pddl_predicate_model = PddlPredicateModel.objects.order_by("name")
        pddl_predicate_dto_list = []

        for ele in pddl_predicate_model:
            pddl_predicate_dto = self._model_to_dto(ele)
            pddl_predicate_dto_list.append(pddl_predicate_dto)

        return pddl_predicate_dto_list

    def _save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """save a PddlPredicateDto
            if the PddlPredicateDto is already saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_predicate_dto):
            return False

        pddl_predicate_model = self._dto_to_model(pddl_predicate_dto)

        # propagating saving
        for pddl_type_model in pddl_predicate_model.types:
            pddl_type_model.save()

        if pddl_predicate_model:
            pddl_predicate_model.save(cascade=True)
            return True

        return False

    def _update(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """update a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to update

        Returns:
            bool: succeed
        """

        pddl_predicate_model = self._get_model(pddl_predicate_dto)

        # check if predicate exists
        if pddl_predicate_model:
            new_pddl_predicate_model = self._dto_to_model(pddl_predicate_dto)
            pddl_predicate_model.name = new_pddl_predicate_model.name
            pddl_predicate_model.types = new_pddl_predicate_model.types
            pddl_predicate_model.save()

            return True

        return False

    def save(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """save or update a PddlPredicateDto
            if the PddlPredicateDto is not saved it will be saved, else it will be updated

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to save or update

        Returns:
            bool: succeed
        """

        if self._exist_in_mongo(pddl_predicate_dto):
            return self._update(pddl_predicate_dto)

        return self._save(pddl_predicate_dto)

    def delete(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """delete a PddlPredicateDto
            if the PddlPredicateDto is not saved return False, else return True

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto to delete

        Returns:
            bool: succeed
        """

        pddl_predicate_model = self._get_model(pddl_predicate_dto)

        # check if predicate exists
        if pddl_predicate_model:
            pddl_predicate_model.delete()
            return True

        return False

    def delete_all(self) -> bool:
        """delete all pddl predicates

        Returns:
            bool: succeed
        """

        pddl_predicate_dto_list = self.get_all()

        for ele in pddl_predicate_dto_list:
            self.delete(ele)

        return True
