import os
from glob import glob
from setuptools import setup

package_name = 'ros_graph_export'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f"{package_name}.templates"],
    package_data={'ros_graph_export': ['templates/*.j2']},
    include_package_data=True,
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                (os.path.join('share', package_name), ['package.xml']),
                (os.path.join('share', package_name,
                              'launch'), glob('launch/*launch.[pxy][yma]*')),
                (os.path.join('share', package_name,
                              'config'), glob('config/*')),
                (os.path.join('share', package_name,
                              'templates'), glob('ros_graph_export/templates/*'))],
    install_requires=['setuptools', 'Jinja2', 'matplotlib'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['ros_graph_export = ros_graph_export.ros_graph_export:main'],
    },
)
