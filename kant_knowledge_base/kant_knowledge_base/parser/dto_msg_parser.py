
""" Dto Msg Parser """

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


class DtoMsgParser:
    """ Dto Msg Parser Class """

    def type_dto_to_msg(self, pddl_type_dto: PddlTypeDto) -> PddlType:
        """ convert a PddlTypeDto into a PddlType msg

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            PddlType: PddlType msg
        """

        msg = PddlType()

        msg.name = pddl_type_dto.get_name()

        return msg

    def object_dto_to_msg(self, pddl_object_dto: PddlObjectDto) -> PddlObject:
        """ convert a PddlObjectDto into a PddlObject msg

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            PddlObject: PddlObject msg
        """

        msg = PddlObject()

        msg.type = self.type_dto_to_msg(pddl_object_dto.get_type())
        msg.name = pddl_object_dto.get_name()

        return msg

    def predicate_dto_to_msg(self, pddl_predicate_dto: PddlPredicateDto) -> PddlPredicate:
        """ convert a PddlPredicateDto into a PddlPredicate msg

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            PddlPredicate: PddlPredicate msg
        """

        msg = PddlPredicate()

        msg.name = pddl_predicate_dto.get_name()

        msg.types = []
        for pddl_type_dto in pddl_predicate_dto.get_types():
            msg.types.append(self.type_dto_to_msg(pddl_type_dto))

        return msg

    def proposition_dto_to_msg(self, pddl_proposition_dto: PddlPropositionDto) -> PddlProposition:
        """ convert a PddlPropositionDto into a PddlProposition msg

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            PddlProposition: PddlProposition msg
        """

        msg = PddlProposition()

        msg.predicate = self.predicate_dto_to_msg(
            pddl_proposition_dto.get_predicate())

        msg.is_goal = pddl_proposition_dto.get_is_goal()

        msg.objects = []
        for pddl_object_dto in pddl_proposition_dto.get_objects():
            msg.objects.append(self.object_dto_to_msg(pddl_object_dto))

        return msg

    def condition_effect_dto_to_msg(self,
                                    pddl_condition_effect_dto:
                                        PddlConditionEffectDto) -> PddlConditionEffect:
        """ convert a PddlConditionEffectDto into a PddlConditionEffect msg

        Args:
            pddl_condition_effect_dto (PddlConditionEffectDto): PddlConditionEffectDto

        Returns:
            PddlConditionEffect: PddlConditionEffect msg
        """

        msg = PddlConditionEffect()

        msg.predicate = self.predicate_dto_to_msg(
            pddl_condition_effect_dto.get_predicate())

        msg.is_negative = pddl_condition_effect_dto.get_is_negative()
        msg.time = pddl_condition_effect_dto.get_time()

        msg.objects = []
        for pddl_object_dto in pddl_condition_effect_dto.get_objects():
            msg.objects.append(self.object_dto_to_msg(pddl_object_dto))

        return msg

    def action_dto_to_msg(self, pddl_action_dto: PddlActionDto) -> PddlAction:
        """ convert a PddlActionDto into a PddlAction msg

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            PddlAction: PddlAction msg
        """

        msg = PddlAction()

        msg.name = pddl_action_dto.get_name()
        msg.duration = pddl_action_dto.get_duration()
        msg.durative = pddl_action_dto.get_durative()

        msg.parameters = []
        for pddl_object_dto in pddl_action_dto.get_parameters():
            msg.parameters.append(self.object_dto_to_msg(pddl_object_dto))

        msg.coditions = []
        for pddl_condition_dto in pddl_action_dto.get_conditions():
            msg.coditions.append(
                self.condition_effect_dto_to_msg(pddl_condition_dto))

        msg.effects = []
        for pddl_effect_dto in pddl_action_dto.get_effects():
            msg.effects.append(
                self.condition_effect_dto_to_msg(pddl_effect_dto))

        return msg
