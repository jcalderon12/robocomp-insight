import subprocess
import shutil
import argparse
from pathlib import Path
from jinja2 import Template

# Setup paths
BASE_DIR = Path(__file__).parent.resolve() # "bullshit_publisher/src/"
TEMPLATES_DIR = BASE_DIR / "templates"

# Templates
def load_template(template_name: str) -> Template:
    with open(TEMPLATES_DIR / template_name, "r", encoding="utf-8") as f:
        return Template(f.read())

cdsl_template = load_template("agent_generator_cdsl_template")
specific_worker_template = load_template("agent_generator_specificworker_template")
config_template = load_template("agent_generator_config_template")

def generate_agent(cause_name: str, output_path: str) -> bool:
    base_dir = Path(output_path).resolve()
    out_dir = (base_dir / cause_name).resolve()

    try:
        # Validate cause_name to prevent empty or malicious input
        if not cause_name or not cause_name.strip() or any(c in cause_name for c in ['/', '\\', ':', '*', '?', '"', '<', '>', '|']):
            raise ValueError(f"Invalid cause name provided: '{cause_name}'")
        
        # Validate output_path to ensure it's a subdirectory of the base directory
        if not out_dir.is_relative_to(base_dir) or out_dir == base_dir:
            raise ValueError(f"Output path must be a subdirectory of the base directory: '{base_dir}'")

        # Delete whole folder and all files inside using syscalls
        if out_dir.exists() and out_dir.is_dir():
            shutil.rmtree(out_dir)

        # Create necessary directories
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "src").mkdir(parents=True, exist_ok=True)
        (out_dir / "etc").mkdir(parents=True, exist_ok=True)

        # Generate cdsl    
        with open( out_dir / f"concept_{cause_name}.cdsl", "w", encoding="utf-8") as f:
            f.write(cdsl_template.render(concept_name=cause_name))

        # Compile cdsl
        subprocess.run(["robocompdsl", f"concept_{cause_name}.cdsl", "."], cwd=out_dir, check=True)
        subprocess.run(["cmake", "-B", "build"], cwd=out_dir, check=True)
        subprocess.run(["make", "-j8", "-C", "build"], cwd=out_dir, check=True)

        # Generate specificworker.py
        with open(out_dir / "src" / "specificworker.py", "w", encoding="utf-8") as f:
            f.write(specific_worker_template.render(concept_name=cause_name))

        # Generate config file
        with open(out_dir / "etc" / "config", "w", encoding="utf-8") as f:
            f.write(config_template.render(concept_name=cause_name))

        return True

    except subprocess.CalledProcessError as e:
        print(f"Subprocess error: {e}")
        return False    

    except Exception as e:
        print(f"Error generating agent: {e}")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a new agent based on a cause name.")
    parser.add_argument("cause_name", type=str, help="The name of the cause for the agent.")
    parser.add_argument("output_path", type=str, help="The output path where the agent will be generated.")

    args = parser.parse_args()

    print(f"Starting generation for agent: '{args.cause_name}' in '{args.output_path}'...")

    success = generate_agent(args.cause_name, args.output_path)
    if success:
        print(f"Agent '{args.cause_name}' generated successfully in '{args.output_path}'.")
        exit(0)
    else:
        print(f"Failed to generate agent '{args.cause_name}'.")
        exit(1)