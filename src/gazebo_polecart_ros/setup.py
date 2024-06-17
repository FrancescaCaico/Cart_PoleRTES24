import os
from itertools import chain
from setuptools import find_packages, setup

package_name = 'gazebo_polecart_ros'

def generate_data_files(dirs=['launch', 'config', 'gazebo/worlds', 'gazebo/models']):
    """
    Generate recursive list of data files, without listing directories in the output.
    """
    data_files = []
    for path, _, files in chain.from_iterable(os.walk(dir) for dir in dirs):
        install_dir = path[len('gazebo/'):] if path.startswith('gazebo/') else path  # remove gazebo/ prefix
        install_dir = os.path.join('share', package_name, install_dir)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carmelo',
    maintainer_email='43755732+cscribano@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'gazebo_spawn = gazebo_polecart_ros.gazebo_spawn:main',
        ],
    },
)
