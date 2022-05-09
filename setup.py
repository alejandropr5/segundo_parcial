import os
from glob import glob
from setuptools import setup

package_name = 'segundo_parcial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejandro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_sub_and_process = segundo_parcial.sub_and_process_img:main',
            'img_segment_pub = segundo_parcial.pub_segmentation:main',            
        ],
    },
)
