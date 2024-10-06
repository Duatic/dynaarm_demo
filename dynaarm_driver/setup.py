from setuptools import find_packages, setup
import os

package_name = 'dynaarm_driver'

def recursive_data_files(directory):
    data_files = []
    for dirpath, dirnames, filenames in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(dirpath, filename)
            data_files.append((os.path.join('share', package_name, dirpath), [full_path]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])] + 
        recursive_data_files('launch') +
        recursive_data_files('config')
    ,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
