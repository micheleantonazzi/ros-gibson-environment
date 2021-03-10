from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import yaml
import os

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_gibson_environment'],
    package_dir={'': 'src'},
)

setup(**setup_args)

# Create configuration files

# Package config
config_parameters = dict()

config_parameters['gibson_config'] = {'model_id': '', 'is_discrete': False, 'random': {'random_initial_pose': False, 'random_init_z_range': [-0.1, 0.1], 'random_target_pose': False, 'random_init_x_range': [-0.1, 0.1], 'random_init_y_range': [-0.1, 0.1], 'random_init_rot_range': [-0.1, 0.1]}, 'ui_num': 3, 'envname': 'TurtlebotNavigateEnv', 'target_orn': [0, 0, 1.57], 'speed': {'timestep': 0.00417, 'frameskip': 10}, 'initial_pos': [0, 0, 0], 'verbose': False, 'fov': 1.57, 'display_ui': True, 'resolution': 512, 'semantic_source': 2, 'target_pos': [0, 0, 0], 'use_filler': True, 'semantic_color': 2, 'mode': 'gui', 'output': ['nonviz_sensor', 'rgb_filled', 'depth', 'semantics'], 'ui_components': ['RGB_FILLED', 'DEPTH', 'SEMANTICS'], 'initial_orn': [0, 0, 0], 'show_diagnostics': False}

with open(os.path.join(os.path.dirname(__file__), 'config', "package_config.yaml"), mode='w') as package_config_file:
    yaml.dump(config_parameters, package_config_file, default_flow_style=False)

