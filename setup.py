from setuptools import find_packages, setup
from pathlib import Path
package_name = 'hri_face_speaking_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/pal_system_module',
         ['module/' + package_name]),
        ('share/ament_index/resource_index/pal_configuration.' + package_name,
            ['config/' + package_name]),
        ('share/' + package_name + '/config', ['config/00-defaults.yml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/module',
         ['module/hri_face_speaking_detector_module.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/face_speaking_detector.launch.py']),
        ('share/' + package_name + '/models',
            ["models/RF_16_1.pkl"]),
        # ('share/' + package_name + '/test/data',
        #    [str(path) for path in Path('test/data').glob('**/*') if path.is_file()]),

    ],
    install_requires=['setuptools', 'hri_face_detect'],
    zip_safe=True,
    maintainer='Sara Cooper',
    maintainer_email='sara.cooper@pal-robotics.com',
    description='HRI Face speaking detector',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hri_face_speaking_detector = hri_face_speaking_detector.hri_face_speaking_detector:main',
        ],
    },
)
