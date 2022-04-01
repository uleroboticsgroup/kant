
""" Mongo models"""

import mongoengine


class PddlTypeModel(mongoengine.Document):
    """ pddl type model """

    meta = {"collection": "pddl_type"}
    name = mongoengine.StringField(primary_key=True)
    _aux = mongoengine.StringField(default="AUX")


class PddlObjectModel(mongoengine.Document):
    """ pddl object model """

    meta = {"collection": "pddl_object"}
    name = mongoengine.StringField(primary_key=True)
    type = mongoengine.ReferenceField(
        PddlTypeModel, reverse_delete_rule=mongoengine.CASCADE)


class PddlPredicateModel(mongoengine.Document):
    """ pddl predicate model """

    meta = {"collection": "pddl_predicate"}
    name = mongoengine.StringField(primary_key=True)
    types = mongoengine.ListField(
        mongoengine.ReferenceField(PddlTypeModel,
                                   reverse_delete_rule=mongoengine.CASCADE))


class PddlPropositionModel(mongoengine.Document):
    """ pddl proposition model """

    meta = {"collection": "pddl_proposition"}
    predicate = mongoengine.ReferenceField(
        PddlPredicateModel, reverse_delete_rule=mongoengine.CASCADE)
    pddl_objects = mongoengine.ListField(
        mongoengine.ReferenceField(
            PddlObjectModel, reverse_delete_rule=mongoengine.CASCADE),
        db_field="objects")
    is_goal = mongoengine.BooleanField()


class PddlParameterModel(mongoengine.EmbeddedDocument):
    """ pddl parameter model """

    meta = {"collection": "pddl_parameter"}
    name = mongoengine.StringField()
    type = mongoengine.ReferenceField(PddlTypeModel)


class PddlConditionEffectModel(mongoengine.EmbeddedDocument):
    """ pddl contion/effect model """

    meta = {"collection": "pddl_condition_effect"}
    time = mongoengine.StringField()
    is_negative = mongoengine.BooleanField()
    predicate = mongoengine.ReferenceField(PddlPredicateModel)
    parameters = mongoengine.EmbeddedDocumentListField(PddlParameterModel)


class PddlActionModel(mongoengine.Document):
    """ pddl action model """

    meta = {"collection": "pddl_action"}

    action_name = mongoengine.StringField(primary_key=True)
    duration = mongoengine.IntField(default=10)
    durative = mongoengine.BooleanField(default=True)

    _predicates = mongoengine.ListField(
        mongoengine.ReferenceField(PddlPredicateModel, reverse_delete_rule=mongoengine.CASCADE))

    parameters = mongoengine.EmbeddedDocumentListField(PddlParameterModel)

    conditions = mongoengine.EmbeddedDocumentListField(
        PddlConditionEffectModel)
    effects = mongoengine.EmbeddedDocumentListField(
        PddlConditionEffectModel)
