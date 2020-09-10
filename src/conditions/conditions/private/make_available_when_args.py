from typing import List, Dict, Union
import yaml
import argparse
import logging
from argparse import RawTextHelpFormatter

example_arg = "[\n{\n\tcondition_name : has_been_greeted,\n\tscheme : allof,\n\trequired_states : [\n\t{\n\t\ttopic_name : chatter1,\n\t\ttopic_type : std_msgs/msg/String,\n\t\trequired_values : {data : hello}\n\t},\n\t{\n\t\ttopic_name : chatter2,\n\t\ttopic_type : std_msgs/msg/String,\n\t\trequired_values : {data : hello}\n\t}]\n}]"

class TopicAndRequiredValue:
    def __init__(self, topic_name : str, topic_type : str, required_values : Dict[str,str]):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.required_values = required_values

class Condition:
    def __init__(self, name : str, scheme : str, required_states : List[TopicAndRequiredValue]):
        self.name = name
        self.scheme = scheme
        self.required_states = required_states

class MakeAvailableWhenArgs:
    conditions : List[Condition] = []
    def add_condition(self, name : str, scheme : str, required_states : List[TopicAndRequiredValue]):
        self.conditions.append(Condition(name, scheme, required_states))

def make_available_when_args(inval : str) -> MakeAvailableWhenArgs:
    """
    Utility function for the argparse `type` parameter. checks that the YAML string passed is valid, and returns an
    equivelent MakeAvailableWhenArgs instance
    """

    args = MakeAvailableWhenArgs()

    values_list = yaml.safe_load(inval)

    if not isinstance(values_list, list):
        logging.error('The passed value needs to be a list in YAML format: Got {}'.format(str(type(values_list))))
        raise ValueError()

    if len(values_list) == 0:
        logging.error('The passed list must contain at least one dictionary')
        raise ValueError()

    for value_entry in values_list:

        if not isinstance(value_entry, dict):
            logging.error('The passed list needs to contain dictionaries in YAML format: Got \"{}\"'.format(str(type(value_entry))))
            raise ValueError()

        value : Dict[str, Union[str,List]] = value_entry
        required = ["condition_name", "scheme", "required_states"]
        if not all(key in value.keys() for key in required):
            logging.error('The top level YAML dictionarys must contain the keys: {}: Got {}'.format(str(required), str(list(value.keys()))))
            raise ValueError()

        condition_name = value["condition_name"]
        if not isinstance(condition_name, str):
            logging.error('The condition_name must be a string')
            raise ValueError()

        scheme = value["scheme"]
        if not isinstance(scheme, str):
            logging.error('The scheme must be a string')
            raise ValueError()

        scheme_options = ["allof", "anyof", "exany"]
        if scheme not in scheme_options:
            logging.error('scheme must be one of {}'.format(str(scheme_options)))
            raise ValueError()

        required_states = value["required_states"]
        if not isinstance(required_states, list):
            logging.error('The required_states value needs to contain a list of dictionaries in YAML format: Got {}'.format(str(type(required_states))))
            raise ValueError()


        if len(required_states) == 0:
            logging.error('The passed list of required_states must contain at least one dictionary')
            raise ValueError()

        topics_and_required_values : List[TopicAndRequiredValue] = []

        for required_state in required_states:

            if not isinstance(required_state, dict):
                logging.error('The required_states list needs to contain dictionaries in YAML format: Got {}'.format(str(type(required_state))))
                raise ValueError()

            required = ["topic_name", "topic_type", "required_values"]
            if not all(key in required_state.keys() for key in required):
                logging.error('The required_states YAML dictionarys must contain the keys: {}: Got {}'.format(str(required), str(list(required_state.keys()))))
                raise ValueError()

            topic_name : str = required_state["topic_name"]
            if not isinstance(topic_name, str):
                logging.error('The topic_name must be a string')
                raise ValueError()

            topic_type : str = required_state["topic_type"]
            if not isinstance(topic_type, str):
                logging.error('The topic_type must be a string')
                raise ValueError()

            required_values : Dict[str,str] = required_state["required_values"]
            if not isinstance(required_values, dict):
                logging.error('The required_values must be a dictionary in YAML format: Got {}'.format(str(type(required_values))))
                raise ValueError()

            topics_and_required_values.append(TopicAndRequiredValue(topic_name, topic_type, required_values))

        args.add_condition(condition_name, scheme, topics_and_required_values)
            
    return args

#TEST
if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter)
    parser.add_argument('--test', required=True, type=make_available_when_args, help='A list of conditions in YAML format e.g \"{}\"'.format(example_arg))
    args = parser.parse_args()
    margs : MakeAvailableWhenArgs = args.test
    for cond in margs.conditions:
        print("condition name {}".format(cond.name))
        print("condition scheme {}".format(cond.scheme))
        for r in cond.required_states:
            print("\ttopic name {}".format(r.topic_name))
            print("\ttopic type {}".format(r.topic_type))
            print("\trequired_values {}".format(r.required_values))