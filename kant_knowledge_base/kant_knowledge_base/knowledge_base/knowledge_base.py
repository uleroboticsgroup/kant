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


""" Knowledge Base """

from typing import List

from kant_dto import (
    PddlTypeDto,
    PddlObjectDto,
    PddlPredicateDto,
    PddlPropositionDto,
    PddlActionDto,
    PddlConditionEffectDto
)


class KnowledgeBase:
    """ Knowledge Base Class """

    def __init__(self):
        self.types_dict = {}
        self.objects_dict = {}
        self.predicates_dict = {}
        self.actions_dict = {}
        self.goals_list = []
        self.propositions_list = []

    def get_type(self, type_name: str) -> PddlTypeDto:
        """ get the PddlTypeDto with a given type name

        Args:
            type_name (str): type name

        Returns:
            PddlTypeDto: PddlTypeDto
        """

        if type_name in self.types_dict:
            return self.types_dict[type_name]

        return None

    def get_all_types(self) -> List[PddlTypeDto]:
        """ get all PddlTypeDto

        Returns:
            List[PddlTypeDto]: list of PddlTypeDto
        """

        type_list = []

        for type_name in sorted(self.types_dict):
            type_list.append(self.types_dict[type_name])

        return type_list

    def save_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ save or update a pddl type

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        type_name = pddl_type_dto.get_name()

        if not type_name in self.types_dict:
            self.types_dict[type_name] = pddl_type_dto
        else:
            self.types_dict[type_name].set_name(pddl_type_dto.get_name())

        return True

    def delete_type(self, pddl_type_dto: PddlTypeDto) -> bool:
        """ delete a pddl type
            if not exists, returns false

        Args:
            pddl_type_dto (PddlTypeDto): PddlTypeDto

        Returns:
            bool: succeed
        """

        if not pddl_type_dto.get_name() in self.types_dict:
            return False

        # propagating deleting
        for pddl_object_dto in self.get_all_objects():
            if pddl_object_dto.get_type() == pddl_type_dto:
                if not self.delete_object(pddl_object_dto):
                    return False

        for pddl_predicate_dto in self.get_all_predicates():
            for pddl_pred_type_dto in pddl_predicate_dto.get_types():
                if pddl_pred_type_dto == pddl_type_dto:
                    if not self.delete_predicate(pddl_predicate_dto):
                        return False

                    break

        for pddl_action_dto in self.get_all_actions():
            for pddl_parameter_dto in pddl_action_dto.get_parameters():
                if pddl_parameter_dto.get_type() == pddl_type_dto:
                    if not self.delete_action(pddl_action_dto):
                        return False

                    break

        del self.types_dict[pddl_type_dto.get_name()]

        return True

    def delete_all_types(self) -> bool:
        """ delete a all pddl types

        Returns:
            bool: succeed
        """

        for pddl_type_dto in self.get_all_types():
            if not self.delete_type(pddl_type_dto):
                return False

        return True

    def get_object(self, object_name: str) -> PddlObjectDto:
        """ get the PddlObjectDto with a given object name

        Args:
            object_name (str): object name

        Returns:
            PddlObjectDto: PddlObjectDto
        """

        if object_name in self.objects_dict:
            return self.objects_dict[object_name]

        return None

    def get_all_objects(self) -> List[PddlObjectDto]:
        """ get all PddlObjectDto

        Returns:
            List[PddlObjectDto]: list of PddlObjectDto
        """

        object_list = []

        for object_name in sorted(self.objects_dict):
            object_list.append(self.objects_dict[object_name])

        return object_list

    def save_object(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ save or update a pddl object

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: succeed
        """

        # propagating saving
        if not self.save_type(pddl_object_dto.get_type()):
            return False

        object_name = pddl_object_dto.get_name()

        pddl_object_dto.set_type(self.get_type(
            pddl_object_dto.get_type().get_name()))

        if not object_name in self.objects_dict:
            self.objects_dict[object_name] = pddl_object_dto
        else:
            self.objects_dict[object_name].set_name(pddl_object_dto.get_name())
            self.objects_dict[object_name].set_type(pddl_object_dto.get_type())

        return True

    def delete_object(self, pddl_object_dto: PddlObjectDto) -> bool:
        """ delete a pddl object
            if not exists, returns false

        Args:
            pddl_object_dto (PddlObjectDto): PddlObjectDto

        Returns:
            bool: succeed
        """

        if not pddl_object_dto.get_name() in self.objects_dict:
            return False

        # propagating deleting
        for pddl_proposition_dto in self.get_all_propositions():
            for pddl_prop_object_dto in pddl_proposition_dto.get_objects():
                if pddl_prop_object_dto == pddl_object_dto:
                    if not self.delete_proposition(pddl_proposition_dto):
                        return False

                    break

        del self.objects_dict[pddl_object_dto.get_name()]

        return True

    def delete_all_objects(self) -> bool:
        """ delete a all pddl objects

        Returns:
            bool: succeed
        """

        for pddl_object_dto in self.get_all_objects():
            if not self.delete_object(pddl_object_dto):
                return False

        return True

    def get_predicate(self, predicate_name: str) -> PddlPredicateDto:
        """ get the PddlPredicateDto with a given predicate name

        Args:
            predicate_name (str): predicate name

        Returns:
            PddlPredicateDto: PddlPredicateDto
        """

        if predicate_name in self.predicates_dict:
            return self.predicates_dict[predicate_name]

        return None

    def get_all_predicates(self) -> List[PddlPredicateDto]:
        """ get all PddlPredicateDto

        Returns:
            List[PddlPredicateDto]: list of PddlPredicateDto
        """

        predicate_list = []

        for predicate_name in sorted(self.predicates_dict):
            predicate_list.append(self.predicates_dict[predicate_name])

        return predicate_list

    def save_predicate(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ save or update a pddl predicate

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: succeed
        """

        # propagating saving
        pddl_type_dto_list = []
        for pddl_type_dto in pddl_predicate_dto.get_types():
            if not self.save_type(pddl_type_dto):
                return False

            pddl_type_dto_list.append(self.get_type(
                pddl_type_dto.get_name()))

        pddl_predicate_dto.set_types(pddl_type_dto_list)

        self.predicates_dict[pddl_predicate_dto.get_name(
        )] = pddl_predicate_dto

        return True

    def delete_predicate(self, pddl_predicate_dto: PddlPredicateDto) -> bool:
        """ delete a pddl predicate
            if not exists, returns false

        Args:
            pddl_predicate_dto (PddlPredicateDto): PddlPredicateDto

        Returns:
            bool: succeed
        """

        if not pddl_predicate_dto.get_name() in self.predicates_dict:
            return False

        # propagating deleting
        for pddl_proposition_dto in self.get_all_propositions():
            if pddl_proposition_dto.get_predicate() == pddl_predicate_dto:
                if not self.delete_proposition(pddl_proposition_dto):
                    return False

        for pddl_action_dto in self.get_all_actions():
            for pddl_condi_effect_dto in (pddl_action_dto.get_conditions() +
                                          pddl_action_dto.get_effects()):
                if pddl_condi_effect_dto.get_predicate() == pddl_predicate_dto:
                    if not self.delete_action(pddl_action_dto):
                        return False

                    break

        del self.predicates_dict[pddl_predicate_dto.get_name()]

        return True

    def delete_all_predicates(self) -> bool:
        """ delete a all pddl predicates

        Returns:
            bool: succeed
        """

        for pddl_predicate_dto in self.get_all_predicates():
            if not self.delete_predicate(pddl_predicate_dto):
                return False

        return True

    def get_action(self, action_name: str) -> PddlActionDto:
        """ get the PddlActionDto with a given action name

        Args:
            action_name (str): action name

        Returns:
            PddlActionDto: PddlActionDto
        """

        if action_name in self.actions_dict:
            return self.actions_dict[action_name]

        return None

    def get_all_actions(self) -> List[PddlActionDto]:
        """ get all PddlActionDto

        Returns:
            List[PddlActionDto]: list of PddlActionDto
        """

        action_list = []

        for action_name in sorted(self.actions_dict):
            action_list.append(self.actions_dict[action_name])

        return action_list

    def _propagate_condition_effect_save(self,
                                         pddl_condi_effect_dto: PddlConditionEffectDto,
                                         pddl_action_dto: PddlActionDto) -> bool:
        """ propagate saving of conditions/effects and check if they are correct.
            a condition/effect is correct if its objects types are the same as the
            types of its predicate, if its objects are action parameters, and if the time
            and durative are correct

        Args:
            pddl_condi_effect_dto (PddlConditionEffectDto): conditions/effects

        Returns:
            bool: are conditions/effects correct?
        """

        # checking condition/effect

        # checking time and durative
        if (not pddl_action_dto.get_durative() and pddl_condi_effect_dto.get_time()):
            return False
        elif (pddl_action_dto.get_durative() and not pddl_condi_effect_dto.get_time()):
            return False

        # checking objects
        pddl_object_dto_list = pddl_condi_effect_dto.get_objects()
        pddl_type_dto_list = pddl_condi_effect_dto.get_predicate().get_types()

        # checking len
        if len(pddl_object_dto_list) != len(pddl_type_dto_list):
            return False

        # checking types and parameters
        for pddl_object_dto, pddl_type_dto in zip(pddl_object_dto_list, pddl_type_dto_list):

            if pddl_object_dto.get_type() != pddl_type_dto:
                return False

            if not pddl_object_dto in pddl_action_dto.get_parameters():
                return False

        # propagating saving of condition/effect

        # condition/effect predicate
        succ = self.save_predicate(
            pddl_condi_effect_dto.get_predicate())

        if not succ:
            return False

        pddl_condi_effect_dto.set_predicate(self.get_predicate(
            pddl_condi_effect_dto.get_predicate().get_name()))

        # condition/effect objects
        for pddl_object_dto in pddl_object_dto_list:
            succ = self.save_type(pddl_object_dto.get_type())

            if not succ:
                return False

            pddl_object_dto.set_type(self.get_type(
                pddl_object_dto.get_type().get_name()))

        return True

    def save_action(self, pddl_action_dto: PddlActionDto) -> bool:
        """ save or update a pddl action

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            bool: succeed
        """

        # propagating saving of parameter
        for pddl_parameter_dto in pddl_action_dto.get_parameters():
            if not self.save_type(pddl_parameter_dto.get_type()):
                return False

            pddl_parameter_dto.set_type(self.get_type(
                pddl_parameter_dto.get_type().get_name()))

        # conditions and effects
        for pddl_condi_effect_dto in (pddl_action_dto.get_conditions() +
                                      pddl_action_dto.get_effects()):
            if not self._propagate_condition_effect_save(
                    pddl_condi_effect_dto, pddl_action_dto):
                return False

        self.actions_dict[pddl_action_dto.get_name(
        )] = pddl_action_dto

        return True

    def delete_action(self, pddl_action_dto: PddlActionDto) -> bool:
        """ delete a pddl action
            if not exists, returns false

        Args:
            pddl_action_dto (PddlActionDto): PddlActionDto

        Returns:
            bool: succeed
        """

        if not pddl_action_dto.get_name() in self.actions_dict:
            return False

        del self.actions_dict[pddl_action_dto.get_name()]

        return True

    def delete_all_actions(self) -> bool:
        """ delete a all pddl actions

        Returns:
            bool: succeed
        """

        self.actions_dict = {}

        return True

    def get_propositions(self, predicate_name: str) -> List[PddlPropositionDto]:
        """ get the PddlPropositionDto with a given predicate name

        Args:
            action_name (str): action name

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        proposition_list = []

        for proposition in self.goals_list + self.get_propositions_no_goals():
            if proposition.get_predicate().get_name() == predicate_name:
                proposition_list.append(proposition)

        return proposition_list

    def get_propositions_goals(self) -> List[PddlPropositionDto]:
        """ get all goal PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self.goals_list

    def get_propositions_no_goals(self) -> List[PddlPropositionDto]:
        """ get all no goal PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self.propositions_list

    def get_all_propositions(self) -> List[PddlPropositionDto]:
        """ get all PddlPropositionDto

        Returns:
            List[PddlPropositionDto]: list of PddlPropositionDto
        """

        return self.goals_list + self.propositions_list

    def _check_proposition(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ check if a proposition (of condition/effect) is correct
            comparing the types of its objects with the types of its
            predicate

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: is proposition correct?
        """

        pddl_object_dto_list = pddl_proposition_dto.get_objects()
        pddl_type_dto_list = pddl_proposition_dto.get_predicate().get_types()

        if len(pddl_object_dto_list) != len(pddl_type_dto_list):
            return False

        for pddl_object_dto, pddl_type_dto in zip(pddl_object_dto_list, pddl_type_dto_list):
            if pddl_object_dto.get_type() != pddl_type_dto:
                return False

        return True

    def save_proposition(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ save or update a pddl action

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: succeed
        """

        if not self._check_proposition(pddl_proposition_dto):
            return False

        # propagating saving

        if not self.save_predicate(
                pddl_proposition_dto.get_predicate()):
            return False

        pddl_proposition_dto.set_predicate(self.get_predicate(
            pddl_proposition_dto.get_predicate().get_name()))

        pddl_object_dto_list = []
        for pddl_object_dto in pddl_proposition_dto.get_objects():
            if not self.save_object(pddl_object_dto):
                return False

            pddl_object_dto_list.append(self.get_object(
                pddl_object_dto.get_name()))

        pddl_proposition_dto.set_objects(pddl_object_dto_list)

        if pddl_proposition_dto.get_is_goal():
            self.goals_list.append(pddl_proposition_dto)

        else:
            self.propositions_list.append(pddl_proposition_dto)

        return True

    def delete_proposition(self, pddl_proposition_dto: PddlPropositionDto) -> bool:
        """ delete a pddl proposition
            if not exists, returns false

        Args:
            pddl_proposition_dto (PddlPropositionDto): PddlPropositionDto

        Returns:
            bool: succeed
        """

        if pddl_proposition_dto.get_is_goal():
            if not pddl_proposition_dto in self.goals_list:
                return False

            self.goals_list.remove(pddl_proposition_dto)

        else:
            if not pddl_proposition_dto in self.propositions_list:
                return False

            self.propositions_list.remove(pddl_proposition_dto)

        return True

    def delete_all_propositions(self) -> bool:
        """ delete a all pddl propositions

        Returns:
            bool: succeed
        """

        self.goals_list = []
        self.propositions_list = []

        return True
