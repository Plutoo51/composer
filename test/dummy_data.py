from muto_msgs.srv import ComposePlugin
import muto_msgs.msg
import muto_msgs.srv
from muto_msgs.msg import PluginResponse, PlanManifest, StackManifest

edge_device = {
   "twin":[
      {
         "twin_url":"http://ditto:ditto@sandbox.composiv.ai",
         "publisher":None,
         "namespace":"org.eclipse.muto.sandbox",
         "name":"example-01",
         "thingId":"org.eclipse.muto.sandbox:example-01",
         "requested_stacks":[
            
         ]
      }
   ],
   "definition":None,
   "state":"unknown",
   "current_stack":None,
   "launcher":[
      
   ]
}

test_manifest = {
   "name":"RVIZ Talker-Listener Stack",
   "context":"eteration_office",
   "stackId":"org.eclipse.muto.sandbox:rviz_talker_listener",
   "node":[
      {
         "name":"talker",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      },
      {
         "name":"listener",
         "pkg":"demo_nodes_cpp",
         "exec":"listener"
      }
   ]
}

other_manifest = {
   "name":"test_stack_name",
   "context":"test_context",
   "stackId":"org.eclipse.muto.sandbox:test_stack_id",
   "node":[
      {
         "name":"talker",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      },
      {
         "name":"listener",
         "pkg":"demo_nodes_cpp",
         "exec":"listener"
      }
   ]
}

main_stack_node =[
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"talker",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"talker",
      "plugin":"",
      "lifecycle":"",
      "name":"talker",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"",
      "action":"",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   }
]

child_stack_node1 = [
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"listener",
         "pkg":"demo_nodes_cpp",
         "exec":"listener"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"listener",
      "plugin":"deneme",
      "lifecycle":"",
      "name":"listener",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"",
      "action":"",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   }
]

child_stack_node2 = [
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"test",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"test",
         "pkg":"test",
         "exec":"test"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"listener",
      "plugin":"test",
      "lifecycle":"test",
      "name":"listener",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"test",
      "action":"test",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   }
]

expected_test_flatten_nodes = [
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"talker",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"talker",
      "plugin":"",
      "lifecycle":"",
      "name":"talker",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"",
      "action":"",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   },
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"listener",
         "pkg":"demo_nodes_cpp",
         "exec":"listener"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"listener",
      "plugin":"deneme",
      "lifecycle":"",
      "name":"listener",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"",
      "action":"",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   },
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"test",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"test",
         "pkg":"test",
         "exec":"test"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"listener",
      "plugin":"test",
      "lifecycle":"test",
      "name":"listener",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"test",
      "action":"test",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   }
]

expected_test_toShallowManifest = {
   "name":"",
   "context":"",
   "stackId":"",
   "param":[
      
   ],
   "arg":[
      
   ],
   "stack":[
      
   ],
   "composable":[
      
   ],
   "node":[
      
   ]
}

test_toManifest = {
   "name":"sam",
   "context":"",
   "stackId":"samet-333",
   "param":[
      "test_param"
   ],
   "arg":[
      
   ],
   "stack":[
      
   ],
   "composable":[
      
   ],
   "node":[
      
   ]
}

expected_test_toManifest = {
   "name":"sam",
   "context":"",
   "stackId":"samet-333",
   "param":[
      
   ],
   "arg":[
      
   ],
   "stack":[
      
   ],
   "composable":[
      
   ],
   "node":[
      
   ]
}

stack_obj_manifest_test_merge ={
   "name":"test_name",
   "context":"test_context",
   "stackId":"test_stackId",
   "node":[
      {
         "name":"RVIZ Talker-Listener Stack",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      }
   ]
}

child_stack_obj_manifest_test_merge = {
   "name":"RVIZ Talker-Listener Stack",
   "context":"eteration_office",
   "stackId":"org.eclipse.muto.sandbox:rviz_talker_listener",
   "node":[
      {
         "name":"RVIZ Client-Server Stack",
         "pkg":"demo_nodes_cpp",
         "exec":"add_two_ints_client"
      }
   ]
}

expected_manifest_test_merge_1 = {
   "name":"RVIZ Talker-Listener Stack",
   "context":"eteration_office",
   "stackId":"org.eclipse.muto.sandbox:rviz_talker_listener",
   "param":[
      
   ],
   "arg":[
      
   ],
   "stack":[
      
   ],
   "composable":[
      
   ],
   "node":[
      {
         "env":[
            
         ],
         "param":[
            
         ],
         "remap":[
            
         ],
         "pkg":"demo_nodes_cpp",
         "lifecycle":"",
         "exec":"talker",
         "plugin":"",
         "name":"RVIZ Talker-Listener Stack",
         "ros_args":"",
         "args":"",
         "namespace":"",
         "launch-prefix":None,
         "output":"both",
         "if":"",
         "unless":"",
         "action":"stop"
      },
      {
         "env":[
            
         ],
         "param":[
            
         ],
         "remap":[
            
         ],
         "pkg":"demo_nodes_cpp",
         "lifecycle":"",
         "exec":"add_two_ints_client",
         "plugin":"",
         "name":"RVIZ Client-Server Stack",
         "ros_args":"",
         "args":"",
         "namespace":"",
         "launch-prefix":None,
         "output":"both",
         "if":"",
         "unless":"",
         "action":"start"
      }
   ]
}

expected_manifest_test_merge_2 = {
   "name":"RVIZ Talker-Listener Stack",
   "context":"eteration_office",
   "stackId":"org.eclipse.muto.sandbox:rviz_talker_listener",
   "param":[
      
   ],
   "arg":[
      
   ],
   "stack":[
      
   ],
   "composable":[
      
   ],
   "node":[
      {
         "env":[
            
         ],
         "param":[
            
         ],
         "remap":[
            
         ],
         "pkg":"demo_nodes_cpp",
         "lifecycle":"",
         "exec":"add_two_ints_client",
         "plugin":"",
         "name":"RVIZ Client-Server Stack",
         "ros_args":"",
         "args":"",
         "namespace":"",
         "launch-prefix":None,
         "output":"both",
         "if":"",
         "unless":"",
         "action":"start"
      },
      {
         "env":[
            
         ],
         "param":[
            
         ],
         "remap":[
            
         ],
         "pkg":"demo_nodes_cpp",
         "lifecycle":"",
         "exec":"talker",
         "plugin":"",
         "name":"RVIZ Talker-Listener Stack",
         "ros_args":"",
         "args":"",
         "namespace":"",
         "launch-prefix":None,
         "output":"both",
         "if":"",
         "unless":"",
         "action":"stop"
      }
   ]
}

expected_edge_device_test_merge = {
   "twin":[
      {
         "twin_url":"http://ditto:ditto@sandbox.composiv.ai",
         "publisher":None,
         "namespace":"org.eclipse.muto.sandbox",
         "name":"example-01",
         "thingId":"org.eclipse.muto.sandbox:example-01",
         "requested_stacks":[
            
         ]
      }
   ],
   "definition":None,
   "state":"unknown",
   "current_stack":None,
   "launcher":[
      
   ]
}

main_composables_manifest = {'name': 'sam', 'context': '', 'stackId': 'org.eclipse.muto.sandbox:samet_333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Client",
                "name": "client",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

child_composables_manifest = {'name': 'sam1', 'context': '', 'stackId': 'org.eclipse.muto.sandbox:samet_333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Listener",
                "name": "listener",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

identify_different_stacks = {
  "thingId": "org.eclipse.muto.sandbox:f1tenth_simulation",
  "policyId": "org.eclipse.muto.sandbox:f1tenth_simulation",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {
    "type": "simulator"
  },
  "features": {
    "stack": {
      "properties": {
        "name": "F1TENTH Simulation",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:f1tenth-simulation",
        "arg": [
          {
            "name": "map",
            "value": "$(find f1tenth_gym_ros)/maps/levine_blocked.yaml"
          }
        ],
        "node": [
          {
            "name": "bridge",
            "pkg": "f1tenth_gym_ros",
            "exec": "gym_bridge",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/config/sim.yaml"
              }
            ]
          },
          {
            "name": "rviz2",
            "pkg": "rviz2",
            "exec": "rviz2",
            "args": "-d $(find f1tenth_gym_ros)/launch/gym_bridge.rviz"
          },
          {
            "name": "map_server",
            "pkg": "nav2_map_server",
            "exec": "map_server",
            "param": [
              {
                "name": "yaml_filename",
                "value": "$(arg map)"
              },
              {
                "name": "topic",
                "value": "map"
              },
              {
                "name": "frame_id",
                "value": "map"
              },
              {
                "name": "output",
                "value": "screen"
              },
              {
                "name": "use_sim_time",
                "value": "True"
              }
            ]
          },
          {
            "name": "lifecycle_manager_localization",
            "pkg": "nav2_lifecycle_manager",
            "exec": "lifecycle_manager",
            "param": [
              {
                "name": "use_sim_time",
                "value": "True"
              },
              {
                "name": "autostart",
                "value": "True"
              },
              {
                "name": "node_names",
                "value": [
                  "map_server"
                ]
              }
            ]
          },
          {
            "name": "ego_robot_state_publisher",
            "pkg": "robot_state_publisher",
            "exec": "robot_state_publisher",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/launch/ego_racecar.xacro"
              }
            ],
            "remap": [
              {
                "from": "/robot_description",
                "to": "ego_robot_description"
              }
            ]
          }
        ]
      }
    }
  }
}

identify_different_stacks_test = {
  "thingId": "org.eclipse.muto.sandbox:test-333",
  "policyId": "org.eclipse.muto.sandbox:test-333",
  "definition": "org.eclipse.muto:EdgeDevice:0.0.1",
  "attributes": {
    "brand": "Tofas",
    "model": "ATT"
  },
  "features": {
    "context": {
      "properties": {
        "name": "Test"
      }
    },
    "stack": {
      "properties": {
        "name": "Test"
      }
    },
    "telemetry": {
      "properties": {
        "name": "Test",
        "definition": []
      }
    }
  }
}

main_compare_and_mark_nodes = {'name': 'sam', 'context': '', 'stackId': 'samet-333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Client",
                "name": "client",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

child_compare_and_mark_nodes = {'name': 'sam', 'context': '', 'stackId': 'samet-333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Listener",
                "name": "listener",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

flatten_composable_main = [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              }
            ]
          }]

flatten_composable_child1 = [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }]

flatten_composable_child2 = [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }]

flatten_composable_expected = [
   {
      "name":"muto_demo_container",
      "namespace":"composition",
      "package":"rclcpp_components",
      "executable":"component_container",
      "node":[
         {
            "pkg":"composition",
            "plugin":"composition::Server",
            "name":"server",
            "namespace":"composable"
         }
      ]
   },
   {
      "name":"muto_demo_container",
      "namespace":"composition",
      "package":"rclcpp_components",
      "executable":"component_container",
      "node":[
         {
            "pkg":"composition",
            "plugin":"composition::Server",
            "name":"server",
            "namespace":"composable"
         },
         {
            "pkg":"composition",
            "plugin":"composition::Talker",
            "name":"talker",
            "namespace":"composable"
         }
      ]
   },
   {
      "name":"muto_demo_container",
      "namespace":"composition",
      "package":"rclcpp_components",
      "executable":"component_container",
      "node":[
         {
            "pkg":"composition",
            "plugin":"composition::Talker",
            "name":"talker",
            "namespace":"composable"
         }
      ]
   }
]

node_for_should_node_run = [
   {
      "stack":[
         {
            "manifest":{
               
            },
            "parent":None,
            "edge_device":None,
            "name":"",
            "context":"",
            "stackId":"",
            "param":[
               
            ],
            "arg":{
               
            },
            "stack":[
               
            ],
            "node":[
               
            ],
            "composable":[
               
            ]
         }
      ],
      "container":None,
      "manifest":{
         "name":"talker",
         "pkg":"demo_nodes_cpp",
         "exec":"talker"
      },
      "env":[
         
      ],
      "param":[
         
      ],
      "remap":[
         
      ],
      "pkg":"demo_nodes_cpp",
      "exec":"talker",
      "plugin":"",
      "lifecycle":"",
      "name":"talker",
      "ros_args":"",
      "args":"",
      "namespace":"",
      "launch_prefix":None,
      "output":"both",
      "iff":"",
      "unless":"",
      "action":"",
      "ros_params":[
         
      ],
      "remap_args":[
         
      ]
   }
]

deneme_compos = [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Client",
                "name": "client",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }]

deneme_compos2 = [{
            "name": "muto_demo_container",
            "namespace": "",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Listener",
                "name": "listener",
                "namespace": "composable"
              }
            ]
          }]

main_compare_composable = {'name': 'sam', 'context': '', 'stackId': 'org.eclipse.muto.sandbox:samet_333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Server",
                "name": "server",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Client",
                "name": "client",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

child_compare_composable = {'name': 'sam1', 'context': '', 'stackId': 'org.eclipse.muto.sandbox:samet_333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container",
            "namespace": "composition",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Listener",
                "name": "listener",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

child_compare_composable_added_removed = {'name': 'sam1', 'context': '', 'stackId': 'org.eclipse.muto.sandbox:samet_333', 'param': [], 'arg': [], 'stack': [], 'composable': [{
            "name": "muto_demo_container_added",
            "namespace": "composition_added",
            "package": "rclcpp_components",
            "executable": "component_container",
            "node": [
              {
                "pkg": "composition",
                "plugin": "composition::Talker",
                "name": "talker",
                "namespace": "composable"
              },
              {
                "pkg": "composition",
                "plugin": "composition::Listener",
                "name": "listener",
                "namespace": "composable"
              }
            ]
          }], 'node': []}

composable_equal_manifest = {
            "package": "test_package",
            "executable": "test_executable",
            "name": "test_name",
            "namespace": "test_namespace",
        }

composable_equal_manifest_other = {
            "package": "muto",
            "executable": "test_executable",
            "name": "muto",
            "namespace": "test_namespace",
        }

properties_test = {
      "properties": {
        "name": "F1TENTH Simulation",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:f1tenth-simulation",
        "arg": [
          {
            "name": "map",
            "value": "$(find f1tenth_gym_ros)/maps/levine_blocked.yaml"
          }
        ],
        "node": [
          {
            "name": "bridge",
            "pkg": "f1tenth_gym_ros",
            "exec": "gym_bridge",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/config/sim.yaml"
              }
            ]
          },
          {
            "name": "rviz2",
            "pkg": "rviz2",
            "exec": "rviz2",
            "args": "-d $(find f1tenth_gym_ros)/launch/gym_bridge.rviz"
          },
          {
            "name": "map_server",
            "pkg": "nav2_map_server",
            "exec": "map_server",
            "param": [
              {
                "name": "yaml_filename",
                "value": "$(arg map)"
              },
              {
                "name": "topic",
                "value": "map"
              },
              {
                "name": "frame_id",
                "value": "map"
              },
              {
                "name": "output",
                "value": "screen"
              },
              {
                "name": "use_sim_time",
                "value": "True"
              }
            ]
          },
          {
            "name": "lifecycle_manager_localization",
            "pkg": "nav2_lifecycle_manager",
            "exec": "lifecycle_manager",
            "param": [
              {
                "name": "use_sim_time",
                "value": "True"
              },
              {
                "name": "autostart",
                "value": "True"
              },
              {
                "name": "node_names",
                "value": [
                  "map_server"
                ]
              }
            ]
          },
          {
            "name": "ego_robot_state_publisher",
            "pkg": "robot_state_publisher",
            "exec": "robot_state_publisher",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/launch/ego_racecar.xacro"
              }
            ],
            "remap": [
              {
                "from": "/robot_description",
                "to": "ego_robot_description"
              }
            ]
          }
        ]
      }
    }

expected_properties = {
        "name": "F1TENTH Simulation",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:f1tenth-simulation",
        "arg": [
          {
            "name": "map",
            "value": "$(find f1tenth_gym_ros)/maps/levine_blocked.yaml"
          }
        ],
        "node": [
          {
            "name": "bridge",
            "pkg": "f1tenth_gym_ros",
            "exec": "gym_bridge",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/config/sim.yaml"
              }
            ]
          },
          {
            "name": "rviz2",
            "pkg": "rviz2",
            "exec": "rviz2",
            "args": "-d $(find f1tenth_gym_ros)/launch/gym_bridge.rviz"
          },
          {
            "name": "map_server",
            "pkg": "nav2_map_server",
            "exec": "map_server",
            "param": [
              {
                "name": "yaml_filename",
                "value": "$(arg map)"
              },
              {
                "name": "topic",
                "value": "map"
              },
              {
                "name": "frame_id",
                "value": "map"
              },
              {
                "name": "output",
                "value": "screen"
              },
              {
                "name": "use_sim_time",
                "value": "True"
              }
            ]
          },
          {
            "name": "lifecycle_manager_localization",
            "pkg": "nav2_lifecycle_manager",
            "exec": "lifecycle_manager",
            "param": [
              {
                "name": "use_sim_time",
                "value": "True"
              },
              {
                "name": "autostart",
                "value": "True"
              },
              {
                "name": "node_names",
                "value": [
                  "map_server"
                ]
              }
            ]
          },
          {
            "name": "ego_robot_state_publisher",
            "pkg": "robot_state_publisher",
            "exec": "robot_state_publisher",
            "param": [
              {
                "from": "$(find f1tenth_gym_ros)/launch/ego_racecar.xacro"
              }
            ],
            "remap": [
              {
                "from": "/robot_description",
                "to": "ego_robot_description"
              }
            ]
          }
        ]
      }

node_test_toManifest = {
   "env":[
      
   ],
   "param":[
      
   ],
   "remap":[
      
   ],
   "pkg":"",
   "lifecycle":"",
   "exec":"",
   "plugin":"",
   "name":"listener",
   "ros_args":"",
   "args":"",
   "namespace":"composable",
   "launch-prefix":None,
   "output":"both",
   "if":"",
   "unless":"",
   "action":"start",
   "lifecycle":"start"
}

muto_composer_pipelines = [
            {
                "name": "start",
                "pipeline": {"step": "action"},
                "compensation": {'service': 'muto_start_stack', 'plugin': 'ComposePlugin'}
            },
            {
                "name": "kill",
                "pipeline": {"step": "step_action"},
                "compensation": {'service': 'muto_kill_stack', 'plugin': 'ComposePlugin'}

            },
            {
                "name": "apply",
                "pipeline": {"step": "step_action"},
                "compensation": {'service': 'muto_apply_stack', 'plugin': 'ComposePlugin'}
            }
        ]

test_bootstrap_twin_assert_with_config= {
   "name":"example-01",
   "namespace":"org.eclipse.muto.sandbox",
   "stack_topic":"stack",
   "twin_topic":"twin",
   "anonymous":False,
   "twin_url":"http://ditto:ditto@sandbox.composiv.ai"
}

test_handle_stack_operation_with = {
  'name': 'RVIZ Talker-Listener Stack', 
  'context': 'eteration_office', 
  'stackId': 'org.eclipse.muto.sandbox:rviz_talker_listener', 
  'param': [], 
  'arg': [], 
  'stack': [], 
  'composable': [], 
  'node': [{
    'env': [], 
    'param': [], 
    'remap': [], 
    'pkg': 'demo_nodes_cpp', 
    'lifecycle': '', 
    'exec': 'add_two_ints_server', 
    'plugin': '', 
    'name': 'add_two_ints_server', 
    'ros_args': '', 
    'args': '', 
    'namespace': '', 
    'launch-prefix': None, 
    'output': 'both', 
    'if': '', 
    'unless': '', 
    'action': 'stop'}, 
           {'env': [], 
            'param': [], 
            'remap': [], 
            'pkg': 'demo_nodes_cpp', 
            'lifecycle': '', 
            'exec': 'listener', 
            'plugin': '', 
            'name': 'listener', 
            'ros_args': '', 
            'args': '', 
            'namespace': '', 
            'launch-prefix': None, 
            'output': 'both', 
            'if': '', 
            'unless': '', 
            'action': 'start'}, 
           {'env': [], 
            'param': [], 
            'remap': [], 
            'pkg': 'demo_nodes_cpp', 
            'lifecycle': '', 
            'exec': 'talker', 
            'plugin': '', 
            'name': 'talker', 
            'ros_args': '', 
            'args': '', 
            'namespace': '', 
            'launch-prefix': None, 
            'output': 'both', 
            'if': '', 
            'unless': '', 
            'action': 'start'}, 
           {'env': [], 
            'param': [], 
            'remap': [], 
            'pkg': 'demo_nodes_cpp', 
            'lifecycle': '', 
            'exec': 'add_two_ints_client', 
            'plugin': '', 
            'name': 'add_two_ints_client', 
            'ros_args': '', 
            'args': '', 
            'namespace': '', 
            'launch-prefix': None, 
            'output': 'both', 
            'if': '', 
            'unless': '', 
            'action': 'stop'
            }]}

test_handle_apply_req_input = PlanManifest(pipeline='apply', current=StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=PluginResponse(result_code=0, error_message='', error_description='')), next=StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=PluginResponse(result_code=0, error_message='', error_description='')), planned=StackManifest(type='', stack='', result=PluginResponse(result_code=0, error_message='', error_description='')), result=PluginResponse(result_code=0, error_message='', error_description=''))

test_handle_start_req_input = PlanManifest(pipeline='activate', current=StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=PluginResponse(result_code=0, error_message='', error_description='')), next=StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=PluginResponse(result_code=0, error_message='', error_description='')), planned=StackManifest(type='', stack='', result=PluginResponse(result_code=0, error_message='', error_description='')), result=PluginResponse(result_code=0, error_message='', error_description=''))

test_handle_stack_operation_req_input = muto_msgs.msg.PlanManifest(pipeline='kill', current=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Talker-Listener Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_talker_listener", "param": [], "arg": [], "stack": [], "composable": [], "node": [{"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_server", "plugin": "", "name": "add_two_ints_server", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "listener", "plugin": "", "name": "listener", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "talker", "plugin": "", "name": "talker", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_client", "plugin": "", "name": "add_two_ints_client", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))

test_handle_stack_operation_error_req_input = muto_msgs.msg.PlanManifest(pipeline='kill', current=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))

test_handle_kill_req_input = PlanManifest(pipeline='kill', current=StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=PluginResponse(result_code=0, error_message='', error_description='')), next=StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=PluginResponse(result_code=0, error_message='', error_description='')), planned=StackManifest(type='', stack='', result=PluginResponse(result_code=0, error_message='', error_description='')), result=PluginResponse(result_code=0, error_message='', error_description=''))

test_success_response_input = muto_msgs.msg.PlanManifest(pipeline='kill', current=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Client-Server Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_client_server", "node": [{"name": "add_two_ints_client", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_client"}, {"name": "add_two_ints_server", "pkg": "demo_nodes_cpp", "exec": "add_two_ints_server"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='json', stack='{"stackId": "org.eclipse.muto.sandbox:rviz_talker_listener"}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Talker-Listener Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_talker_listener", "param": [], "arg": [], "stack": [], "composable": [], "node": [{"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_server", "plugin": "", "name": "add_two_ints_server", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "listener", "plugin": "", "name": "listener", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "talker", "plugin": "", "name": "talker", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_client", "plugin": "", "name": "add_two_ints_client", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))

test_success_response_expected_value = muto_msgs.msg.PlanManifest(pipeline='', current=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='json', stack='{"name": "RVIZ Talker-Listener Stack", "context": "eteration_office", "stackId": "org.eclipse.muto.sandbox:rviz_talker_listener", "param": [], "arg": [], "stack": [], "composable": [], "node": [{"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_server", "plugin": "", "name": "add_two_ints_server", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "listener", "plugin": "", "name": "listener", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "talker", "plugin": "", "name": "talker", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "start"}, {"env": [], "param": [], "remap": [], "pkg": "demo_nodes_cpp", "lifecycle": "", "exec": "add_two_ints_client", "plugin": "", "name": "add_two_ints_client", "ros_args": "", "args": "", "namespace": "", "launch-prefix": null, "output": "both", "if": "", "unless": "", "action": "stop"}]}', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description=''))

test_error_response_expected_return = muto_msgs.msg.PlanManifest(pipeline='', current=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), next=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), planned=muto_msgs.msg.StackManifest(type='', stack='', result=muto_msgs.msg.PluginResponse(result_code=0, error_message='', error_description='')), result=muto_msgs.msg.PluginResponse(result_code=1000, error_message='Exception during operation', error_description='Exception during operation'))