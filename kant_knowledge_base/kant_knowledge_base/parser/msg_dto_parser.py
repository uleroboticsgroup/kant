# Copyright (C) 2023  Miguel Ángel González Santamarta

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


""" Msg Dto Parser """

from kant_msgs.msg import PddlType
from kant_msgs.msg import PddlObject
from kant_msgs.msg import PddlPredicate
from kant_msgs.msg import PddlProposition
from kant_msgs.msg import PddlConditionEffect
from kant_msgs.msg import PddlAction

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlConditionEffectDto,
    PddlActionDto
)


class MsgDtoParser:
    """ Msg Dto Parser Class """

    def type_msg_to_dto(self, pddl_type_msg: PddlType) -> PddlTypeDto:
        """ convert a PddlType msg into a PddlTypeDto

        Args:
            pddl_type_msg (PddlType): PddlType msg

        Returns:
            PddlTypeDto: PddlTypeDto
        """

        dto = PddlTypeDto(pddl_type_msg.name)

        return dto

    def object_msg_to_dto(self, pddl_object_msg: PddlObject) -> PddlObjectDto:
        """ convert a PddlObject msg into a PddlObjectDto

        Args:
            pddl_object_msg (PddlObject): PddlObject msg

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        dto = PddlObjectDto(
            self.type_msg_to_dto(pddl_object_msg.type),
            pddl_object_msg.name
        )

        return dto

    def predicate_msg_to_dto(self, pddl_predicate_msg: PddlPredicate) -> PddlPredicateDto:
        """ convert a PddlPredicate msg into a PddlPredicateDto

        Args:
            pddl_predicate_msg (PddlPredicate): PddlPredicate msg

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        dto = PddlPredicateDto(pddl_predicate_msg.name)

        pddl_types_list = []

        for pddl_type_msg in pddl_predicate_msg.types:
            pddl_types_list.append(self.type_msg_to_dto(pddl_type_msg))

        dto.set_types(pddl_types_list)

        return dto

    def proposition_msg_to_dto(self, pddl_proposition_msg: PddlProposition) -> PddlPropositionDto:
        """ convert a PddlProposition msg into a PddlPropositionDto

        Args:
            pddl_proposition_msg (PddlProposition): PddlProposition msg

        Returns:
            PddlPropositionDto: PddlPropositionDto
        """

        dto = PddlPropositionDto(
            self.predicate_msg_to_dto(pddl_proposition_msg.predicate)
        )

        dto.set_is_goal(pddl_proposition_msg.is_goal)

        pddl_objects_list = []

        for pddl_object_msg in pddl_proposition_msg.objects:
            pddl_objects_list.append(self.object_msg_to_dto(pddl_object_msg))

        dto.set_objects(pddl_objects_list)

        return dto

    def condition_effect_msg_to_dto(self,
                                    pddl_condition_effect_msg:
                                        PddlConditionEffect) -> PddlConditionEffectDto:
        """ convert a PddlConditionEffect msg into a PddlConditionEffectDto

        Args:
            pddl_condition_effect_msg (PddlConditionEffect): PddlConditionEffect msg

        Returns:
            PddlConditionEffectDto: PddlConditionEffectDto
        """

        dto = PddlConditionEffectDto(
            self.predicate_msg_to_dto(
                pddl_condition_effect_msg.predicate)
        )

        dto.set_is_negative(pddl_condition_effect_msg.is_negative)
        dto.set_time(pddl_condition_effect_msg.time)

        pddl_objects_list = []

        for pddl_object_msg in pddl_condition_effect_msg.objects:
            pddl_objects_list.append(self.object_msg_to_dto(pddl_object_msg))

        dto.set_objects(pddl_objects_list)

        return dto

    def action_msg_to_dto(self, pddl_action_msg: PddlAction) -> PddlActionDto:
        """ convert a PddlAction msg into a PddlActionDto

        Args:
            pddl_action_msg (PddlAction): PddlAction msg

        Returns:
            PddlActionDto: PddlActionDto
        """

        dto = PddlActionDto(pddl_action_msg.name)

        dto.set_duration(pddl_action_msg.duration)
        dto.set_durative(pddl_action_msg.durative)

        pddl_parameters_list = []
        for pddl_object_msg in pddl_action_msg.parameters:
            pddl_parameters_list.append(
                self.object_msg_to_dto(pddl_object_msg))
        dto.set_parameters(pddl_parameters_list)

        pddl_coditions_list = []
        for pddl_condition_msg in pddl_action_msg.coditions:
            pddl_coditions_list.append(
                self.condition_effect_msg_to_dto(pddl_condition_msg))
        dto.set_conditions(pddl_coditions_list)

        pddl_effect_list = []
        for pddl_effect_msg in pddl_action_msg.effects:
            pddl_effect_list.append(
                self.condition_effect_msg_to_dto(pddl_effect_msg))
        dto.set_effects(pddl_effect_list)

        return dto
