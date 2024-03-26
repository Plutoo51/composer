#
#  Copyright (c) 2024 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#

import subprocess
import shlex
import yaml
import traceback
import composer.model.stack as Stack
import composer.model.node as Node
from composer.expression_resolver import ExpressionResolver

class Param:
    def __init__(self, stack: Stack, manifest=None, node: Node = None):
        self.manifest = manifest or {}
        self.stack = stack
        self.node = node
        self.resolver = ExpressionResolver(stack=stack)
        self.name = manifest.get('name', '')
        self.value = self._resolve_value(self.manifest)
        self.from_file = manifest.get('from', '')
        self.command = manifest.get('command', '')

    def _resolve_value(self, manifest: dict = {}):
        """Resolve the value of the parameter from various sources."""
        value = manifest.get('value', '')
        if manifest.get('from', ''):
            return self._resolve_from_file(self.resolver.resolve_expression(manifest['from']))
        if manifest.get('command', ''):
            return self._execute_command(self.resolver.resolve_expression(manifest['command']))
        if self.resolver.has_expression(value):
            return self._resolve_param_expression(manifest.get('value', ''))
        if isinstance(value, str) \
                or isinstance(value, int) \
                or isinstance(value, float):
            return self._parse_value(value)
        return value

    def _resolve_param_expression(self, value: str | int | float):
        """Resolve param expressions like find, arg, etc."""
        return self.resolver.resolve_expression(value)

    def _resolve_from_file(self, filepath: str):
        """Fetch and return the content of the specified file."""
        try:
            with open(filepath, 'r') as file:
                yaml_contents = yaml.safe_load(file)
                if yaml_contents is None:
                    raise ValueError(
                        "YAML file is empty or contains invalid syntax.")

                matching_key = None
                for key in yaml_contents.keys():
                    if key == f"{self.node.namespace}/{self.node.name}":
                        matching_key = key
                        break

                if matching_key is None:
                    raise ValueError(
                        f"Node name '{self.node.name}' not found in the YAML file.")

                ros_parameters = yaml_contents.get(
                    matching_key, {}).get('ros__parameters', {})
                for key, value in ros_parameters.items():
                    if key is not None and value is not None:
                        self.node.ros_params.append({key: value})

        except FileNotFoundError as e:
            self.stack.nnode.get_logger().error(f"File not found error: {e}")
        except PermissionError as e:
            self.stack.nnode.get_logger().error(
                f"Permission error while reading file '{filepath}': {e}")
        except yaml.YAMLError as e:
            self.stack.nnode.get_logger().error(
                f"YAML error while reading file '{filepath}': {e}")
        except BaseException as e:
            traceback.print_exc()
            self.stack.nnode.get_logger().error(
                f"Failed to read from file '{filepath}': {e}")

    def _execute_command(self, command: str = ""):
        """Execute the specified command and return its output."""
        try:
            return subprocess.check_output(shlex.split(command), text=True).strip()
        except subprocess.CalledProcessError as e:
            self.stack.nnode.get_logger().error(
                f"Command execution failed: {e}")
            return None

    def _parse_value(self, value: str = ""):
        """Parse the given value into the appropriate data type."""
        if isinstance(value, str):
            if value.lower() == 'true':
                return True
            if value.lower() == 'false':
                return False
        if isinstance(value, bool):
            return value
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value

    def toManifest(self):
        """Convert this Param instance into a manifest dictionary."""
        return {
            "name": self.name,
            "value": self.value,
            "from": self.from_file,
            "command": self.command,
        }

    def __eq__(self, other):
        """Check equality based on the attributes of the Param instance."""
        return isinstance(other, Param) and all(
            getattr(self, attr) == getattr(other, attr) for attr in [
                'name', 'value', 'from_file', 'command'
            ])

    def __hash__(self):
        """Generate a hash value for this Param instance."""
        return hash((self.name, self.value, self.from_file, self.command))
