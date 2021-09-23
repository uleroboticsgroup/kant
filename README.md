# KANT (Knowledge mANagemenT)

This is a ROS 2 tool to manage PDDL-based knowledge from Python code. It is based on several software design patterns (DTO, DAO, Factory).

![](./diagram.png)


## Structure
The project structure is the following:

### kant_dao (Knowledge Representation)
ROS 2 package that contains all Python code about PDDL DAO. There are two DAO families implemented:
  - `MONGO`: this is a DAO family that uses MongoDB to storage the PDDL knowledge. Besides, the Mongoengine Python library is used to access MongoDB.
  - `ROS2`: this is a DAO family that uses a ROS 2 node to storage the PDDL knowledge. ROS 2 services are used.


### kant_dto (Knowledge Manipulation)
ROS 2 package that contains all Python code about PDDL DTO. Current PDDL elements that can be used are:
  - types
  - objects
  - predicates
  - propositions
  - goals
  - actions (and durative)


### kant_interfaces
ROS 2 package of the ROS interfaces (msg, srv) used by KANT. They are mainly used to communicate the ROS2 knowledge_base node with other ROS components.

### kant_knowledge_base
ROS 2 package that contains a node that acts as a knowledge base. As a result, PDDL knowledge is stored in memory.

```ros2 run kant_knowledge_base knowledge_base_node```

## Experiments

The In-Memory (ROS2) and MongoDB (MONGO) DAO families have been compared. The results of the performed experiment are presented [here](./Experiments/README.md).
