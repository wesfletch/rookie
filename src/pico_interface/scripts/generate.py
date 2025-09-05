#!/usr/bin/env python3

# Generate the pico_interface python (and eventually cpp) libraries
# based on the input yaml.

# The following inline metadata should allow for 'pipx run generate.py'
# without needing to do any venv/local install shenanigans.
# /// script
# requires-python = ">=3.10.12"
# dependencies = [
#   "black",
#   "datetime",
#   "jinja2",
#   "pyyaml",
# ]
# ///

import datetime
import yaml
from jinja2 import Environment, FileSystemLoader, Template
import black


def generate_py(
    message_definitions: dict,
    env: Environment,
    output_file_name: str,
) -> None:
    """
    Generate the Python lib for pico_interface from message yaml.
    """
    py_template: Template = env.get_template("message.py.j2")
    py_output: str = py_template.render(
        messages=message_definitions["messages"],
        date_time=datetime.datetime.now(),
    )

    # Format the output with `black` so it doesn't look like hot garbage
    formatted_output: str = black.format_str(py_output, mode=black.FileMode())

    with open(output_file_name, "w") as f:
        f.write(formatted_output)


# TODO (maybe)
def generate_cpp() -> None:
    pass


with open("pico_interface.yaml", mode="r") as f:
    message_definitions = yaml.safe_load(f)

env: Environment = Environment(loader=FileSystemLoader("./templates/"))

output_file_name: str = "pico_interface.py"

generate_py(
    message_definitions=message_definitions,
    env=env,
    output_file_name=output_file_name)

print(f"Auto-generation completed. Results at: `{output_file_name}`")