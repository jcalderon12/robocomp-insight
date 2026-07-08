from jinja2 import Template
from instance_generator.instance_simple_random import InstanceSimpleRandom
from instance_generator.instance_random_range_coordinates import InstanceRandomRangeCoordinates
from instance_generator.instance_random_uniform_range import InstanceRandomUniformRange
from instance_generator.instance_none import InstanceNone
from apply_action.apply_action_instantiate_body import ApplyActionInstantiateBody
from apply_action.apply_action_stop_robot_wheel import ApplyActionStopRobotWheel
from apply_action.apply_action_apply_external_force import ApplyActionApplyExternalForce
from apply_action.apply_action_set_friction import ApplyActionSetFriction
from pydantic import BaseModel
from typing import Union
import argparse


"""
    Crea las uniones discriminadas

    Esto le dice a Pydantic: “Cualquier campo de tipo generator puede ser cualquiera de estas implementaciones concretas, el id decide cuál”.

    from typing import Union

    InstanceGeneratorUnion = Union[RandomXGenerator, RandomYGenerator]
    ApplyActionUnion = Union[LogAction]  # añade más acciones aquí"""
InstanceGeneratorUnion = Union[InstanceRandomRangeCoordinates, InstanceSimpleRandom, InstanceRandomUniformRange, InstanceNone]
ApplyActionUnion = Union[ApplyActionInstantiateBody, ApplyActionStopRobotWheel, ApplyActionApplyExternalForce, ApplyActionSetFriction]
CauseTemplate:Template = Template(
    
'''
"""
This cause has been generated.
"""

from causes.cause import Cause
from engines.engine import Engine
from pydantic import BaseModel, PrivateAttr
from typing import Literal

class Cause{{cause_name.capitalize()}}(BaseModel, Cause):
    """{{cause_description}}"""

    name:Literal["{{cause_name.lower()}}"]

    {{instance_generators}}

    {{ apply_variables | join('\n    ') }} 
    def apply(self, engine:Engine):
        {% for call in apply_calls %}
        {{call}}
        {% endfor %}
        pass
    
    {{ compute_variables | join('\n    ') }}
    def apply_compute(self, engine:Engine):
        {% for call in compute_calls %}
        {{call}}
        {% endfor %}
        pass
        
    def get_generated_instances(self):
        return {{ "{" }}
        {% for attr in instance_generator_attributes %}
        "{{attr[0]}}": [{{attr[1]}}],
        {% endfor %}
        {{ "}" }}
'''
    )

def _rendered_to_lines(rendered) -> list[str]:
    """Normalize a render_* result (string or list of strings) into a flat list
    of lines, preserving each line's own indentation."""
    if isinstance(rendered, str):
        chunks = [rendered]
    else:
        chunks = [str(item) for item in rendered]
    lines = []
    for chunk in chunks:
        lines.extend(chunk.splitlines())
    # Drop leading/trailing blank lines but keep inner ones for readability
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    return lines


def _indent_body(text: str) -> str:
    """Indent a column-0 block into the class body (every line but the first)."""
    lines = text.splitlines()
    return "\n".join(lines[:1] + ["    " + line if line.strip() else line for line in lines[1:]])


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
        attributes = []
        for generator in self.instance_generators:
            block = _indent_body(generator.render_generate_instance().strip("\n"))
            instance_generator_code.append(block)
            attributes.append([generator.type, f"self.{generator.id}_{generator.type}()"])

        return "\n\n    ".join(instance_generator_code), attributes

    def render_actions(self):
        prev_vars = []
        calls = []
        for action in self.apply_actions:
            prev_vars.extend(_rendered_to_lines(action.render_action_variables()))
            calls.extend(_rendered_to_lines(action.render_action_call()))
        return prev_vars, calls

    def render_compute_actions(self):
        prev_vars = []
        calls = []
        for action in self.apply_compute_actions:
            prev_vars.extend(_rendered_to_lines(action.render_action_variables()))
            calls.extend(_rendered_to_lines(action.render_action_call()))
        return prev_vars, calls

    def render_cause(self):
        instance_generators, instance_generator_attributes = self.render_instance_generators()
        apply_variables, apply_calls = self.render_actions()
        compute_variables, compute_calls = self.render_compute_actions()
        self.name = self.name.lower()
        return CauseTemplate.render(cause_name=self.name, 
                             cause_description=self.description,
                             instance_generators=instance_generators,
                             instance_generator_attributes=instance_generator_attributes,
                             apply_calls=apply_calls,
                             apply_variables=apply_variables,
                             compute_calls=compute_calls,
                             compute_variables=compute_variables,
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