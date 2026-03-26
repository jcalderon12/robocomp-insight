import os
from jinja2 import Template

# Agent generator templates

cdsl_template:Template = None
with open("src/agent_generator_cdsl_template") as f:
    cdsl_template = Template(f.read())

specific_worker_template:Template = None
with open("src/agent_generator_specificworker_template") as f:
    specific_worker_template = Template(f.read())
    
config_template:Template = None
with open("src/agent_generator_config_template") as f:
    config_template = Template(f.read())


# Agent generator functions

def generate_cdsl(cause_name:str, agent_path:str) -> bool:
    ofile = open(os.path.join(agent_path, f"concept_{cause_name}.cdsl"), "w")
    ofile.write(cdsl_template.render(concept_name=cause_name))
    ofile.close()
    return True

def compile_cdsl(cause_name:str, agent_path:str) -> bool:
    if os.system(f"robocompdsl {agent_path}/concept_{cause_name}.cdsl {agent_path}/") != 0:
        print("Error compiling CDSL")
        return False
    if os.system(f"cd {agent_path} && cmake -B build && make -j8 -C build") != 0:
        print("Error performing cbuild")
        return False
    return True
    
def generate_specific_worker(cause_name:str, agent_path:str) -> bool:
    ofile = open(os.path.join(agent_path, "src", f"specificworker.py"), "w")
    ofile.write(specific_worker_template.render(concept_name=cause_name))
    ofile.close()
    return True

def generate_config(cause_name:str, agent_path:str) -> bool:
    ofile = open(os.path.join(agent_path, "etc", f"config"), "w")
    ofile.write(config_template.render(concept_name=cause_name))
    ofile.close()
    return True

def generate_agent(cause_name:str, output_path:str) -> bool:
    try:
        # Delete whole folder and all files inside using syscalls
        os.system(f"rm -rf {os.path.join(output_path, cause_name)}")
        os.makedirs(os.path.join(output_path, cause_name), exist_ok=True)
        generate_cdsl(cause_name, os.path.join(output_path, cause_name))
        compile_cdsl(cause_name, os.path.join(output_path, cause_name))
        generate_specific_worker(cause_name, os.path.join(output_path, cause_name))
        generate_config(cause_name, os.path.join(output_path, cause_name))
        return True
    except Exception as e:
        print(f"Error generating agent: {e}")
        return False
    
    
    
    