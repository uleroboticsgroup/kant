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


"""Pddl Condition/Effect Dto"""

from typing import List
from kant_dto.pddl_proposition_dto import PddlPropositionDto
from kant_dto.pddl_predicate_dto import PddlPredicateDto
from kant_dto.pddl_object_dto import PddlObjectDto


class PddlConditionEffectDto(PddlPropositionDto):
    """Pddl Condition/Effect Class Dto"""

    AT_START = "at start"
    AT_END = "at end"
    OVER_ALL = "over all"

    def __init__(
        self,
        pddl_predicate_dto: PddlPredicateDto,
        pddl_objects_list: List[PddlObjectDto] = None,
        time: str = None,
        is_negative: bool = False,
    ):

        self.set_time(time)
        self.set_is_negative(is_negative)

        super().__init__(pddl_predicate_dto, pddl_objects_list)

    def get_time(self) -> str:
        """time getter

        Returns:
            str: time the condition/effect will be resolved
        """

        return self._time

    def set_time(self, time: str):
        """time setter

        Args:
            time (str): time the condition/effect will be resolved
        """

        self._time = time

    def get_is_negative(self) -> bool:
        """is negative getter

        Returns:
            bool: is this condition/effect negative
        """

        return self._is_negative

    def set_is_negative(self, is_negative: bool):
        """is negative setter

        Args:
            is_negative (bool): is this condition/effect negative
        """

        self._is_negative = is_negative

    def __str__(self):

        string = "(" + self.get_predicate().get_name()

        for pddl_object in self.get_objects():
            string += " ?" + pddl_object.get_name()

        string += ")"

        if self._is_negative:
            string = "(not " + string + ")"

        if self._time:
            string = "(" + self._time + " " + string + ")"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlConditionEffectDto):

            if not other.get_predicate() == self.get_predicate():
                return False

            if not len(other.get_objects()) == len(self.get_objects()):
                return False

            if not other.get_is_negative() == self.get_is_negative():
                return False

            if not other.get_time() == self.get_time():
                return False

            for pddl_object, other_pddl_object in zip(
                self.get_objects(), other.get_objects()
            ):
                if not pddl_object == other_pddl_object:
                    return False

            return True

        return False
