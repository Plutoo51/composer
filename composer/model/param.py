#
#  Copyright (c) 2023 Composiv.ai, Eteration A.S. and others
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

class Param:
    def __init__(self, stack, manifest=None, node=None):
        if manifest is None:
            manifest = {}

        self.stack = stack
        self.node = node
        self.manifest = manifest
        self.name = manifest.get('name', '')
        self.value = self._resolve_value(manifest, stack)
        self.sep = manifest.get('sep', '')
        self.from_file = manifest.get('from', '')
        self.namespace = manifest.get('namespace', '/')
        self.command = manifest.get('command', '')

    def _resolve_value(self, manifest, stack):
        """Resolve the value of the parameter from various sources."""
        if 'from' in manifest:
            return self._resolve_from_file(stack.resolve_expression(manifest['from']))
        if 'command' in manifest:
            return self._execute_command(stack.resolve_expression(manifest['command']))
        return self._parse_value(manifest.get('value'))

    def _resolve_from_file(self, filepath):
        """Fetch and return the content of the specified file."""
        try:
            ros_parameters_list = []
            with open(filepath, 'r') as file:
                yaml_contents = yaml.safe_load(file)
                if yaml_contents is None:
                    raise ValueError("YAML file is empty or contains invalid syntax.")

                matching_key = None
                for key in yaml_contents.keys():
                    if key == self.node.name:
                        matching_key = key
                        break

                if matching_key is None:
                    raise ValueError(f"Node name '{self.node.name}' not found in the YAML file.")

                ros_parameters = yaml_contents.get(matching_key, {}).get('ros__parameters', {})
                for key, value in ros_parameters.items():
                    if key is not None and value is not None:
                        self.node.ros_params.append({key: value})

        except FileNotFoundError as e:
            print(f"File not found error: {e}")
        except PermissionError as e:
            print(f"Permission error while reading file '{filepath}': {e}")
        except yaml.YAMLError as e:
            print(f"YAML error while reading file '{filepath}': {e}")
        except BaseException as e:
            traceback.print_exc()
            print(f"Failed to read from file '{filepath}': {e}")
        return None

            

    def _execute_command(self, command):
        """Execute the specified command and return its output."""
        try:
            return subprocess.check_output(shlex.split(command), text=True).strip()
        except subprocess.CalledProcessError as e:
            print(f"Command execution failed: {e}")
            return None

    def _parse_value(self, value):
        """Parse the given value into the appropriate data type."""
        if isinstance(value, str):
            if value.lower() == 'true':
                return True
            if value.lower() == 'false':
                return False
            try:
                return int(value)
            except ValueError:
                try:
                    return float(value)
                except ValueError:
                    return value
        return value

    def toManifest(self):
        """Convert this Param instance into a manifest dictionary."""
        return {
            "name": self.name,
            "value": self.value,
            "sep": self.sep,
            "from": self.from_file,
            "namespace": self.namespace,
            "command": self.command,
        }

    def __eq__(self, other):
        """Check equality based on the attributes of the Param instance."""
        return isinstance(other, Param) and all(
            getattr(self, attr) == getattr(other, attr) for attr in [
                'name', 'value', 'from_file', 'namespace', 'command'
            ])

    def __hash__(self):
        """Generate a hash value for this Param instance."""
        return hash((self.name, self.value, self.from_file, self.namespace, self.command))
