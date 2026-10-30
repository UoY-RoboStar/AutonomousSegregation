import os
from setuptools import setup

package_name = 'CacheCons'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['models/', 'launch/', 'worlds/', 'config/', 'urdf/', 'data/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arjunbadyal',
    maintainer_email='arjun.badyal@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'CacheCons1 = CacheCons.CacheCons1:main',
        'CacheCons2 = CacheCons.CacheCons2:main',
        'CacheCons3 = CacheCons.CacheCons3:main',
        'controller = CacheCons.controller:main',
        'random_walk = CacheCons.random_walk:main',
        'obstacle_avoidance = CacheCons.obstacle_avoidance:main',
        'move_manager = CacheCons.move_manager:main',
        'cache_cons_s = CacheCons.cache_cons_s:main',
        'cache_point_assignment = CacheCons.cache_point_assignment:main',
        'target_watch = CacheCons.target_watch:main',
        'stateMachine1_real = CacheCons.stateMachine1_real:main',
        'stateMachine2_real = CacheCons.stateMachine2_real:main',
        'stateMachine3_real = CacheCons.stateMachine3_real:main',
        'camera_calibrator = CacheCons.camera_calibrator:main',
        'calibration_grid = CacheCons.calibration_grid:main',
        'overhead_pose = CacheCons.overhead_pose:main',
        'item_manager = CacheCons.item_manager:main',
        'simulation_ready = CacheCons.simulation_ready:main',
        'OverheadCamera = CacheCons.OverheadCamera:main',
        ],
    },
)
