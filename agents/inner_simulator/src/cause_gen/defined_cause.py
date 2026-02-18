from jinja2 import Template
from instance_generator.instance_simple_random import InstanceSimpleRandom
from instance_generator.instance_random_range_coordinates import InstanceRandomRangeCoordinates
from instance_generator.instance_none import InstanceNone
from apply_action.apply_action_instantiate_body import ApplyActionInstantiateBody
from apply_action.apply_action_stop_robot_wheel import ApplyActionStopRobotWheel
from pydantic import BaseModel
from typing import Union, Annotated
import argparse


"""
    Crea las uniones discriminadas

    Esto le dice a Pydantic: “Cualquier campo de tipo generator puede ser cualquiera de estas implementaciones concretas, el id decide cuál”.

    from typing import Union

    InstanceGeneratorUnion = Union[RandomXGenerator, RandomYGenerator]
    ApplyActionUnion = Union[LogAction]  # añade más acciones aquí"""
InstanceGeneratorUnion = Union[InstanceRandomRangeCoordinates, InstanceSimpleRandom, InstanceNone]
ApplyActionUnion = Union[ApplyActionInstantiateBody, ApplyActionStopRobotWheel]
CauseTemplate:Template = Template(
    
'''from cause import Cause
from engines.engine import Engine
from pydantic import BaseModel
from typing import Literal

class Cause{{cause_name.capitalize()}}(BaseModel, Cause):
    """{{cause_description}}"""

    name:Literal["{{cause_name.lower()}}"]

    {{instance_generator}}

    {{ apply_variables | join('\n    ') }} 
    def apply(self, engine:Engine):
        {% for call in apply_calls %}
        {{call}}
        {% endfor %}
    
    {{ compute_variables | join('\n    ') }}
    def apply_compute(self, engine:Engine):
        {% for call in compute_calls %}
        {{call}}
        {% endfor %}
'''
    )

class DefinedCause(BaseModel):
    """This class allows users to define new causes and generate the code of them.
    The amount of actions that can be º is limited, so writing your own 
    causes is to be considered in order to have more complex code.
    """
    
    name:str # The cause name
    description:str # The cause description
    instance_generators:list[InstanceGeneratorUnion] # The instance generator to generate the instance of the cause
    apply_actions:list[ApplyActionUnion] # The actions to be applied when the cause is applied.
    apply_compute_actions:list[ApplyActionUnion] # The actions to be applied during the compute.

    def render_instance_generators(self):
        instance_generator_code = []
        for generator in self.instance_generators:
            instance_generator_code.append(generator.render_generate_instance())
        return "\n\n    ".join(instance_generator_code)

    def render_actions(self):
        prev_vars = []
        calls = []
        for action in self.apply_actions:
            for pvar in action.render_action_variables(): prev_vars.append(pvar)
            for call in action.render_action_call(): calls.append(call)
        return prev_vars, calls

    def render_compute_actions(self):
        prev_vars = []
        calls = []
        for action in self.apply_compute_actions:
            for pvar in action.render_action_variables(): prev_vars.append(pvar)
            for call in action.render_action_call(): calls.append(call)
        return prev_vars, calls

    def render_cause(self):
        prev_vars, calls = self.render_actions()
        compute_prev_vars, compute_calls = self.render_compute_actions()
        self.name = self.name.lower()
        return CauseTemplate.render(cause_name=self.name, 
                             cause_description=self.description, 
                             apply_actions=self.render_actions(),
                             compute_actions=self.render_compute_actions(),
                             instance_generator=self.render_instance_generators(),
                             apply_variables=prev_vars, 
                             compute_variables=compute_prev_vars,
                             apply_calls=calls,
                             compute_calls=compute_calls
                             )
    
def main():
    parser = argparse.ArgumentParser(
        prog = "Causes Generator",
        description = "Converts a cause defined in a JSON to Python code.",
        epilog = "RoboLab - 2026"
    )
    
    parser.add_argument('-f', '--file', type=str, required=True, help='The path to the JSON file containing the cause definition.')
    parser.add_argument('-o', '--output', type=str, required=False, help='Output file name.')
    args = parser.parse_args()
    
    file = open(args.file, 'r')
    model = DefinedCause.model_validate_json(file.read())
    json_data = model.render_cause()

    if not args.output:
        outputfile = f"cause_{model.name}.py"
    else:
        outputfile = args.output if args.output.endswith('.py') else f"{args.output}.py"

    with open(outputfile, 'w') as f:
        f.write(json_data)

if __name__ == "__main__":
    main()