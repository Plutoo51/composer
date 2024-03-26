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

from composer.model.stack import Stack
from composer.introspection.launcher import Ros2LaunchParent
from rclpy.node import Node

UNKNOWN = 'unknown'
ACTIVE = 'active'
KILLED = 'killed'


class EdgeDevice:
    def __init__(self, node: Node = None, twin=None):
        self.twin = twin
        self.node = node
        self.launcher = Ros2LaunchParent()
        self.definition = {}
        self.current_stack = Stack(
            node=self.node, manifest={})
        self.state = UNKNOWN

    def bootstrap(self):
        try:
            self.node.get_logger().info("Edge Device boostrapping")
            current_definition = self.twin.get_current_stack().get('current', {})
            stack_id = current_definition.get('stackId')
            self.definition = self.stack(stack_id)
            self.state = current_definition.get('state', UNKNOWN)
            self._update_current_stack(self.definition, self.state)
            self.node.get_logger().info("Edge Device boostrap done")

        except Exception as e:
            self._handle_exception('bootstrapping', e)

    def _update_current_stack(self, definition=None, state=UNKNOWN):
        """Helper method to update the current stack based on the provided definition and state."""
        if definition:
            self.current_stack = Stack(
                node=self.node, manifest=definition)
            self.state = state
            self.twin.set_current_stack(
                self.current_stack, state=self.state)
        else:
            self.node.get_logger().warn(
                "Empty definition or stack in edge device stack updating method.")
            self.current_stack = Stack(node=self.node, manifest={})

    def _handle_exception(self, action, exception):
        """Centralized exception handling."""
        self.node.get_logger().error(
            f'An exception occurred during {action}: {exception}')

    def activate(self, current=None):
        try:
            if self.current_stack:
                self.current_stack.kill_all(self.launcher)
            self._update_current_stack(current, ACTIVE)
            self.current_stack.launch(self.launcher)
        except Exception as e:
            self._handle_exception('activation', e)

    def apply(self, current: Stack):
        try:
            self._update_current_stack(current, ACTIVE)
            self.current_stack.apply(self.launcher)
        except Exception as e:
            self._handle_exception('applying changes', e)

    def kill(self, payload: dict = None):
        try:
            if self.current_stack:
                self.current_stack.kill_all(self.launcher)
            self._update_current_stack(payload, KILLED)
        except Exception as e:
            self._handle_exception('killing process', e)

    def stack(self, stackId):
        """Retrieve stack definition by ID."""
        return self.twin.stack(stackId)
