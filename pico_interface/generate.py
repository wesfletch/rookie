#!/usr/bin/env python3

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


def generate_py(message_definitions: dict, env: Environment) -> None:
    py_template: Template = env.get_template("message.py.j2")
    py_output: str = py_template.render(
        messages=message_definitions["messages"],
        date_time=datetime.datetime.now(),
    )

    # Format the output with black
    formatted_output: str = black.format_str(py_output, mode=black.FileMode())

    with open("pico_interface.py", "w") as f:
        f.write(formatted_output)


with open("pico_interface.yaml", mode="r") as f:
    message_definitions = yaml.safe_load(f)

env: Environment = Environment(loader=FileSystemLoader("./templates/"))

generate_py(message_definitions=message_definitions, env=env)

print("Auto-generation completed.")
