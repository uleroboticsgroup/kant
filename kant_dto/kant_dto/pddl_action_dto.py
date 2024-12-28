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


""" Pddl Action Dto """

from typing import List
from kant_dto.dto import Dto
from kant_dto.pddl_condition_effect_dto import PddlConditionEffectDto
from kant_dto.pddl_object_dto import PddlObjectDto


class PddlActionDto(Dto):
    """ Pddl Action Dto Class """

    def __init__(self, name: str,
                 parameters: List[PddlObjectDto] = None,
                 conditions: List[PddlConditionEffectDto] = None,
                 effects: List[PddlConditionEffectDto] = None,
                 durative: bool = True):

        self.set_name(name)
        self.set_parameters(parameters)
        self.set_durative(durative)
        self.set_duration(10)
        self.set_conditions(conditions)
        self.set_effects(effects)

        Dto.__init__(self)

    def get_name(self) -> str:
        """ pdd action name getter

        Returns:
            str: pddl action name
        """

        return self._name

    def set_name(self, name: str):
        """ pddl action name setter

        Args:
            name (str): pddl action name
        """

        self._name = name

    def get_durative(self) -> bool:
        """ durative getter

        Returns:
            bool: is this a durative action
        """

        return self._durative

    def set_durative(self, durative: bool):
        """ durative setter

        Args:
            durative (bool): is this a durative action
        """

        self._durative = durative

    def get_duration(self) -> int:
        """ duration getter

        Returns:
            int: action duration
        """

        return self._duration

    def set_duration(self, duration: int):
        """ duration setter

        Args:
            duration (int): action duration
        """

        self._duration = duration

    def get_parameters(self) -> List[PddlObjectDto]:
        """ parameters list getter

        Returns:
            List[PddlObjectDto]: list of action parameters
        """

        return self._parameters

    def set_parameters(self, parameters: List[PddlObjectDto]):
        """ parameters list setter

        Args:
            parameters (List[PddlObjectDto]): list of action parameters
        """

        if parameters:
            self._parameters = parameters
        else:
            self._parameters = []

    def get_conditions(self) -> List[PddlConditionEffectDto]:
        """ conditions list getter

        Returns:
            List[PddlConditionEffectDto]: list of action conditions
        """

        return self._conditions

    def set_conditions(self, conditions: List[PddlConditionEffectDto]):
        """ conditions list setter

        Args:
            conditions (List[PddlConditionEffectDto]): list of action conditions
        """

        if conditions:
            self._conditions = conditions
        else:
            self._conditions = []

    def get_effects(self) -> List[PddlConditionEffectDto]:
        """ effects list getter

        Returns:
            List[PddlConditionEffectDto]: list of action effects
        """
        return self._effects

    def set_effects(self, effects: List[PddlConditionEffectDto]):
        """ effects list setter

        Args:
            effects (List[PddlConditionEffectDto]): list of action effects
        """

        if effects:
            self._effects = effects
        else:
            self._effects = []

    def __str__(self):
        string = "(:"

        # durative
        if self._durative:
            string += "durative-"
        string += "action " + self._name

        # parameters
        string += "\n\t:parameters ("
        for parameter in self._parameters:
            string += " ?" + parameter.get_name() + " - " + \
                parameter.get_type().get_name()
        string += ")"

        # duration
        if self._durative:
            string += "\n\t:duration (= ?duration " + str(self._duration) + ")"

        # conditions
        if self._durative:
            string += "\n\t:condition (and"
        else:
            string += "\n\t:precondition (and"
        for condi in self._conditions:
            string += "\n\t\t" + str(condi)
        string += "\n\t)"

        # effects
        string += "\n\t:effect (and"
        for effect in self._effects:
            string += "\n\t\t" + str(effect)
        string += "\n\t)"

        string += "\n)"

        return string

    def __eq__(self, other) -> bool:
        if isinstance(other, PddlActionDto):
            return self.get_name() == other.get_name()

        return False
