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
import rclpy
import composer.model.param as param
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState


class Node:
    def __init__(self, stack, manifest: dict = {}, container=None):
        if manifest is None:
            manifest = {}

        self.stack = stack
        self.container = container
        self.manifest = manifest
        self.env = manifest.get('env', [])
        self.name = manifest.get('name', '')
        self.ros_params = []
        self.namespace = manifest.get('namespace', os.getenv('MUTONS', ''))
        self.param = [param.Param(stack, pDef, self)
                      for pDef in manifest.get('param', [])]
        self.remap = manifest.get('remap', [])
        self.pkg = manifest.get('pkg', '')
        self.exec = manifest.get('exec', '')
        self.plugin = manifest.get('plugin', '')
        self.lifecycle = manifest.get('lifecycle', '')
        self.ros_args = manifest.get('ros_args', '')
        self.args = stack.resolve_expression(manifest.get('args', ''))
        self.launch_prefix = manifest.get('launch-prefix', None)
        self.output = manifest.get('output', 'both')
        self.iff = manifest.get('if', '')
        self.unless = manifest.get('unless', '')
        self.action = manifest.get('action', '')

        for p in self.param:
            if p.name is not None and p.value is not None:
                self.ros_params.append({p.name: p.value})

        self.remap_args = [(stack.resolve_expression(
            rm['from']), stack.resolve_expression(rm['to'])) for rm in self.remap]

    def toManifest(self):
        """Converts the node object back into a manifest dictionary."""
        return {
            "env": self.env,
            "param": [p.toManifest() for p in self.param],
            "remap": [{"from": rm[0], "to": rm[1]} for rm in self.remap_args],
            "pkg": self.pkg,
            "lifecycle": self.lifecycle,
            "exec": self.exec,
            "plugin": self.plugin,
            "name": self.name,
            "ros_args": self.ros_args,
            "args": self.args,
            "namespace": self.namespace,
            "launch-prefix": self.launch_prefix,
            "output": self.output,
            "if": self.iff,
            "unless": self.unless,
            "action": self.action
        }

    def change_state(self, verbs: list=[]):
        if self.lifecycle:
            temporary_node = rclpy.create_node('change_state_node')
            state_cli = temporary_node.create_client(
                ChangeState, f'/{self.namespace}/{self.name}/change_state')
            while not state_cli.wait_for_service(timeout_sec=1.0):
                temporary_node.get_logger().warn(
                    'Lifecycle change state service not available. Waiting...')

            for verb in verbs:
                request = ChangeState.Request()
                t = Transition()
                t.label = verb
                request.transition = t
                future = state_cli.call_async(request)
                rclpy.spin_until_future_complete(
                    temporary_node, future, timeout_sec=3.0)
            temporary_node.destroy_node()
        else:
            self.stack.nnode.get_logger().warn(
                f"{self.name} is Not a managed node")

    def __eq__(self, other):
        """Checks if two Node objects are equal based on their attributes."""
        return isinstance(other, Node) and all(
            getattr(self, attr) == getattr(other, attr) for attr in [
                'pkg', 'name', 'namespace', 'exec', 'plugin', 'args'
            ])

    def __hash__(self):
        """Computes a hash based on certain attributes of the Node."""
        return hash((self.pkg, self.name, self.namespace, self.exec, self.plugin))
