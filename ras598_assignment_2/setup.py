# from setuptools import find_packages, setup

# package_name = 'ras598_assignment_2'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='eva',
#     maintainer_email='sdhir5@asu.edu',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#             'planner_node = ras598_assignment_2.planner_node:main'
#         ],
#     },
# )

from setuptools import find_packages, setup

package_name = 'ras598_assignment_2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        # Install all files that planner_launch.py needs at runtime
        (f'share/{package_name}', ['planner_launch.py']),
        (f'share/{package_name}', ['grading_scout.py']),
        (f'share/{package_name}', ['map.yaml']),
        (f'share/{package_name}', ['cave_filled.png']),
        (f'share/{package_name}', ['planning.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eva',
    maintainer_email='eva@example.com',
    description='RAS 598 Assignment 2: Motion Planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = ras598_assignment_2.planner_node:main',
        ],
    },
)
