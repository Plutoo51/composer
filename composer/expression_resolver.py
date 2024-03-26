
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

import os
import re
import uuid
from ament_index_python.packages import get_package_share_directory


class ExpressionResolver:
    """Resolve Muto expressions like find, arg, etc."""

    def __init__(self,  stack=None) -> None:
        self.stack = stack

    def has_expression(self, value: str = ""):
        """
        Determines if a param value contains expression or not
        Returns True if it contains an expression
        Returns False if it doesn't contain an expression
        """
        return re.search(r'\$\((.*?)\)', str(value)) is not None

    def resolve_expression(self, value: str = ""):
        """Resolve Muto expressions like find, arg, etc.

        Args:
            value (str, optional): The value containing expressions. Defaults to "".

        Returns:
            str: The resolved value.
        """
        if not self.has_expression(value):
            return value

        value = str(value)
        expressions = re.findall(r'\$\(([\s0-9a-zA-Z_-]+)\)', value)
        result = value

        for expression in expressions:
            expr, var = expression.split()
            resolved_value = ""

            try:
                if expr == 'find':
                    resolved_value = get_package_share_directory(var)
                elif expr == 'env':
                    resolved_value = os.environ[var]
                elif expr == 'optenv':
                    resolved_value = os.environ.get(var, '')
                elif expr == 'arg':
                        if self.stack:
                            for a in self.stack.arg:
                                if a.name == var:
                                    resolved_value = a.value
                        else:
                            raise Exception(f"Expression $({expr} {var}) could not be resolved")
                elif expr == 'anon':
                    resolved_value = self.anon.get(var, var + uuid.uuid1().hex)
                    self.anon[var] = resolved_value
                elif expr == 'eval':
                    raise NotImplementedError(
                        f"Value: {value} is not supported in Muto")
                else:
                    continue
                result = re.sub(
                    r'\$\(' + re.escape(expression) + r'\)', resolved_value, result, count=1)
            except KeyError:
                raise Exception(f"{var} does not exist", 'param')
            except Exception as e:
                raise Exception(f'Exception occurred: {e}')

        return result
