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

import composer.model.node as node
import composer.model.param as param
import composer.model.composable as composable
import composer.model.arg as arg
import rclpy
import json
from rclpy.node import Node
from composer.twin import Twin
from composer.introspection.introspector import Introspector
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from composer.expression_resolver import ExpressionResolver

NOACTION = 'none'
STARTACTION = 'start'
STOPACTION = 'stop'
LOADACTION = 'load'


class Stack():
    """The class that contains all stack related operations (apply, kill, stack, merge etc.)"""

    def __init__(self, node: Node, manifest: dict = {}):
        """Initialize the Stack object.

        Args:
            edge_device (object): The edge device object.
            manifest (dict, optional): The manifest dictionary containing stack details. Defaults to {}.
            parent (object, optional): The parent stack object. Defaults to None.
        """

        self.manifest = manifest
        self.nnode = node  # Passed in ros node for logging purposes
        self.twin = Twin(self.nnode, self.nnode.muto)
        self.name = manifest.get('name', '')
        self.context = manifest.get('context', '')
        self.stackId = manifest.get('stackId', '')
        self.param = manifest.get('param', [])
        self.arg = manifest.get('arg', [])
        self.node = []
        self.stack = []
        self.composable = []
        self.resolver = ExpressionResolver()

        if self.name and self.manifest and self.stackId:
            self.initialize()

    def initialize(self):
        """Initialize the stack elements (nodes, composable nodes, parameters etc.)"""
        referenced_stacks = self.manifest.get('stack', [])

        args = []
        for aDef in self.arg:
            args.append(arg.Arg(self, aDef))
        self.arg = args

        params = []
        for pDef in self.param:
            params.append(param.Param(self, pDef))
        self.param = params

        for nDef in self.manifest.get('node', []):
            self.node.append(node.Node(self, nDef))

        for cDef in self.manifest.get('composable', []):
            self.composable.append(
                composable.Container(self, cDef))

        for stackRef in referenced_stacks:
            stackDef = self.twin.stack(stackRef['thingId'])
            stack = Stack(manifest=stackDef)
            self.stack.append(stack)

    def compare_nodes(self, other):
        """Compare the nodes of the stack with another stack.

        Args:
            other (Stack): The other stack object to compare with.

        Returns:
            tuple: A tuple containing sets of common, different, and added nodes.
        """
        nodeSet = set(self.flatten_nodes([]))
        otherNodeSet = set(other.flatten_nodes([]))
        common = nodeSet.intersection(otherNodeSet)
        difference = nodeSet.difference(otherNodeSet)
        added = otherNodeSet.difference(nodeSet)
        return common, difference, added

    def compare_composable(self, other):
        """Compare the composable nodes of the stack with another stack.

        Args:
            other (Stack): The other stack object to compare with.

        Returns:
            tuple: A tuple containing sets of common, different, and added composable nodes.
        """
        current_composables = {
            f"{c.namespace}/{c.name}": c for c in self.flatten_composable([])}
        other_composables = {
            f"{c.namespace}/{c.name}": c for c in other.flatten_composable([])}

        common_keys = current_composables.keys() & other_composables.keys()
        added_keys = other_composables.keys() - current_composables.keys()
        removed_keys = current_composables.keys() - other_composables.keys()

        common = [current_composables[key] for key in common_keys]
        added = [other_composables[key] for key in added_keys]
        removed = [current_composables[key] for key in removed_keys]

        return common, added, removed

    def flatten_nodes(self, list):
        """Flatten the nested structure of nodes in the stack.

        Args:
            list (list): The list to store flattened nodes.

        Returns:
            list: The flattened list of nodes.
        """
        try:
            for n in self.node:
                list.append(n)
            for s in self.stack:
                s.flatten_nodes(list)
            return list
        except Exception as e:
            self.nnode.get_logger().info(
                f'Exception occured in flatten_nodes: {e}')

    def flatten_composable(self, list):
        """Flatten the nested structure of composable nodes in the stack.

        Args:
            list (list): The list to store flattened composable nodes.

        Returns:
            list: The flattened list of composable nodes.
        """

        try:
            for c in self.composable:
                list.append(c)
            for s in self.stack:
                s.flatten_composable(list)
            return list
        except Exception as e:
            self.nnode.get_logger().info(
                f'Exception occured in flatten_composable: {e}')

    def merge(self, other):
        """Merge the current stack with another stack.

        Args:
            other (Stack): The other stack object to merge with.

        Returns:
            Stack: The merged stack object.
        """

        merged = Stack(node=self.nnode,
                       manifest={})
        self._merge_attributes(merged, other)
        self._merge_nodes(merged, other)
        self._merge_composables(merged, other)
        self._merge_params(merged, other)

        merged.manifest = merged.toManifest()
        return merged

    def _merge_attributes(self, merged, other):
        merged.name = other.name
        merged.context = other.context
        merged.stackId = other.stackId

    def _merge_nodes(self, merged, other):
        common, difference, added = self.compare_nodes(other)

        for node in common:
            node.action = NOACTION
        for node in added:
            node.action = STARTACTION
        for node in difference:
            node.action = STOPACTION
        merged.node = common.union(added).union(difference)

    def _merge_composables(self, merged, other):
        merged.composable = []

        current_containers = {(c.namespace, c.name): c for c in self.composable}
        other_containers = {(c.namespace, c.name): c for c in other.composable}

        # Process added and removed containers
        for key, container in other_containers.items():
            if key not in current_containers:
                # Mark all nodes within new containers as STARTACTION
                for node in container.nodes:
                    node.action = STARTACTION
                merged.composable.append(container)
            else:
                # For existing containers, compare nodes within and mark actions
                current_container = current_containers[key]
                self.compare_and_mark_nodes(
                    current_container, container, merged)

        for key, container in current_containers.items():
            if key not in other_containers:
                # Mark all nodes within removed containers as STOPACTION
                for node in container.nodes:
                    node.action = STOPACTION
                merged.composable.append(container)

        return merged

    def compare_and_mark_nodes(self, current_container, other_container, merged):
        current_nodes = {(n.namespace, n.name): n for n in current_container.nodes}
        other_nodes = {(n.namespace, n.name): n for n in other_container.nodes}

        for key, node in other_nodes.items():
            if key not in current_nodes:
                node.action = STARTACTION
            else:
                node.action = NOACTION

        for key, node in current_nodes.items():
            if key not in other_nodes:
                node.action = STOPACTION
            else:

                if node.action != STARTACTION:
                    node.action = NOACTION

        # Add processed nodes back into their respective containers
        processed_container = other_container if other_container in merged.composable else current_container
        processed_container.nodes = list(current_nodes.values(
        )) + [n for n in other_nodes.values() if n.action == STARTACTION]
        if processed_container not in merged.composable:
            merged.composable.append(processed_container)

    def _merge_params(self, merged, other):
        other_params = {param.name: param.value for param in other.param}
        for pn, pv in other_params.items():
            merged.param.append(param.Param(self, {"name": pn, "value": pv}))
        merged.arg = other.arg

    def get_active_nodes(self):
        """Get a list of active nodes.

        Returns:
            list: A list of active nodes.
        """
        n = rclpy.create_node('get_active_nodes')
        n_list = n.get_node_names_and_namespaces()
        n.destroy_node()
        return n_list

    def kill_all(self, launcher):
        """Kill all active nodes which were launched by Muto.

        Args:
            launcher (object): The launcher object.
        """
        intrspc = Introspector()
        for n in launcher._active_nodes:
            for name, pid in n.items():
                intrspc.kill(name, pid)

    def kill_diff(self, launcher, stack):
        """When apply pipeline runs, kill the difference between two stacks.

        Args:
            launcher (object): The launcher object.
            stack (Stack): The stack object.
        """
        intrspc = Introspector()

        # Kill nodes
        for n in stack.node:
            for e in launcher._active_nodes:
                for exec_name, pid in e.items():
                    if n.exec in exec_name and n.action == STOPACTION:
                        intrspc.kill(exec_name, pid)

        # Kill composables
        for container in stack.composable:
            for cn in container.nodes:
                for e in launcher._active_nodes:
                    for exec_name, pid in e.items():
                        if cn.exec in exec_name and cn.action == STOPACTION:
                            intrspc.kill(exec_name, pid)

    def toShallowManifest(self):
        manifest = {"name": self.name,
                    "context": self.context,
                    "stackId": self.stackId,
                    "param": [],
                    "arg": [],
                    "stack": [],
                    "composable": [],
                    "node": []}
        return manifest

    def toManifest(self):
        """Convert the stack to a manifest dictionary.

        Returns:
            dict: Manifest dictionary representing the stack.
        """
        manifest = self.toShallowManifest()

        for s in self.stack:
            manifest["stack"].append(s.toShallowManifest())
        for p in self.param:
            manifest["param"].append(p.toManifest())
        for a in self.arg:
            manifest["arg"].append(a.toManifest())
        for n in self.node:
            manifest["node"].append(n.toManifest())
        for c in self.composable:
            manifest["composable"].append(c.toManifest())

        return manifest

    def process_remaps(self, remaps_config):
        """Process remaps configuration.

        Args:
            remaps_config (list): List of remaps configurations.

        Returns:
            list: List of processed remaps.
        """
        return [(rmp['from'], rmp['to']) for rmp in remaps_config] if remaps_config else []

    def should_node_run(self, node):
        """Check if a node should run. 
        This method clears the situation where a 
        node has NOACTION but it isn't running
        NOACTION is meant to keep the common processes alive when switching stacks

        Args:
            node (object): The node object.

        Returns:
            bool: True if the node should run, False otherwise.
        """
        active_nodes = [(active[1] if active[1] != '/' else '') +
                        '/' + active[0] for active in self.get_active_nodes()]

        should_node_run = f'/{node.namespace}/{node.name}' not in active_nodes
        return should_node_run

    def load_common_composables(self, container, launch_description: LaunchDescription):
        """If there are common containers in stack composables, load them onto the existing container
        Args: 
            container (object): The container object
        """
        node_desc = []
        for cn in container.nodes:
            if cn.action == LOADACTION:
                node_desc.append(ComposableNode(
                    package=cn.pkg,
                    name=cn.name,
                    namespace=cn.namespace,
                    plugin=cn.plugin
                ))

        if node_desc:
            load_action = LoadComposableNodes(
                target_container=f'{container.namespace}/{container.name}',
                composable_node_descriptions=[node_desc],
            )
            launch_description.add_action(load_action)

    def handle_composable_nodes(self, composable_containers, launch_description, launcher):
        """Handle composable nodes during stack launching.

        Args:
            composable_containers (list): List of composable nodes.
            launch_description (object): The launch description object.
        """
        for c in composable_containers:
            node_desc = [ComposableNode(package=cn.pkg, plugin=cn.plugin, name=cn.name, namespace=cn.namespace, parameters=cn.ros_params, remappings=self.process_remaps(cn.remap))
                         for cn in c.nodes if cn.action == STARTACTION or (cn.action == NOACTION and self.should_node_run(cn))]

            if node_desc:  # If node_desc is not empty
                container = ComposableNodeContainer(
                    name=c.name,
                    namespace=c.namespace,
                    package=c.package,
                    executable=c.executable,
                    output=c.output,
                    composable_node_descriptions=node_desc,
                )
                launch_description.add_action(container)

            # self.load_common_composables(c, launch_description)

    def handle_regular_nodes(self, nodes, launch_description, launcher):
        """Handle regular nodes during stack launching.

        Args:
            nodes (list): List of regular nodes.
            launch_description (object): The launch description object.
        """
        for n in nodes:
            if n.action == STARTACTION or (n.action == NOACTION and self.should_node_run(n)):
                launch_description.add_action(Node(
                    package=n.pkg,
                    executable=n.exec,
                    name=n.name,
                    namespace=n.namespace,
                    output=n.output,
                    parameters=n.ros_params,
                    arguments=n.args.split(),
                    remappings=self.process_remaps(n.remap)
                ))

    def handle_managed_nodes(self, nodes, verb):
        """Handle regular nodes during stack launching.

        Args:
            nodes (list): List of lifecycle nodes.
            launch_description (object): The launch description object.
        """
        for n in nodes:
            if n.lifecycle:
                verbs = n.lifecycle.get(verb, [])
                n.change_state(verbs=verbs)

    def launch(self, launcher):
        """Launch the stack.

        Args:
            launcher (Ros2LaunchParent object): The launcher object.
        """
        launch_description = LaunchDescription()

        try:
            self.handle_composable_nodes(
                self.composable, launch_description, launcher)
            self.handle_regular_nodes(self.node, launch_description, launcher)

            launcher.start(launch_description)
            all_nodes = self.node + \
                [cn for c in self.composable for cn in c.nodes]

            # After nodes are launched, take care of managed node actions (configure, activate etc.)
            self.handle_managed_nodes(all_nodes, verb='start')
        except Exception as e:
            self.nnode.get_logger().info(
                f'Stack launching ended with exception: {e}')

    def apply(self, launcher):
        """Apply the stack.

        Args:
            launcher (Ros2LaunchParent object): The launcher object.
        """
        self.kill_diff(launcher, self)
        self.launch(launcher)
