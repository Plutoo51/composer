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

class Arg(object):
    def __init__(self, stack, manifest={}):
        self.stack = stack
        self.name = manifest.get('name', '')
        self.value = self.resolve_arg(manifest.get('value', ''))
        self.default = self.resolve_arg(manifest.get('default', ''))
        self.description = manifest.get('description', '')

    def toManifest(self):
        manifest = {"name": self.name, "value": self.value,
                    "default": self.default, "description": self.description}
        return manifest

    def resolve_arg(self, val):
        if self.stack.has_expression(val):
            return self.stack.resolve_expression(val)
        return val
