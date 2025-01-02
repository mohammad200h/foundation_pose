from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'foundation_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/' + package_name + '/learning/datasets/' , [
          package_name + '/learning/datasets/'+'h5_dataset.py',
          package_name + '/learning/datasets/'+'pose_dataset.py'
        ]),
        ('lib/python3.10/site-packages/' + package_name + '/learning/models/' , [
          package_name + '/learning/models/'+'network_modules.py',
          package_name + '/learning/models/'+'refine_network.py',
          package_name + '/learning/models/'+'score_network.py',

        ]),
        ('lib/python3.10/site-packages/' + package_name + '/learning/training/' , [
           package_name + '/learning/training/'+'predict_pose_refine.py',
           package_name + '/learning/training/'+'predict_score.py',
           package_name + '/learning/training/'+'training_config.py',

        ]),
        ('lib/python3.10/site-packages/' + package_name + '/weights/2023-10-28-18-33-37/' , [
           package_name + '/weights/2023-10-28-18-33-37/config.yml',
           package_name + '/weights/2023-10-28-18-33-37/model_best.pth',
        ]),
        ('lib/python3.10/site-packages/' + package_name + '/weights/2024-01-11-20-02-45/' , [
           package_name + '/weights/2024-01-11-20-02-45/config.yml',
           package_name + '/weights/2024-01-11-20-02-45/model_best.pth',
        ]),
        # mustard0 rgb
        ('lib/python3.10/site-packages/' + package_name +'/demo_data/mustard0/rgb/',[
          package_name + f'/demo_data/mustard0/rgb/{filename}'for filename in
          os.listdir(package_name + f'/demo_data/mustard0/rgb/')
          if filename.endswith('.png')
        ]),
        # mustard0 mesh
        ('lib/python3.10/site-packages/' + package_name +'/demo_data/mustard0/',[
          package_name + '/demo_data/mustard0/mustard0.obj']),
        # mustard0 depth
        ('lib/python3.10/site-packages/' + package_name +'/demo_data/mustard0/depth/',[
          package_name + f'/demo_data/mustard0/depth/{filename}'for filename in
          os.listdir(package_name + f'/demo_data/mustard0/depth/')
          if filename.endswith('.png')
        ]),
        # mustard0 masks
        ('lib/python3.10/site-packages/' + package_name +'/demo_data/mustard0/masks/',[
          package_name + f'/demo_data/mustard0/masks/{filename}'for filename in
          os.listdir(package_name + f'/demo_data/mustard0/masks/')
          if filename.endswith('.png')
        ]),
        # launch files
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*')
        ),

    ],
    include_package_data=True,
    package_data={'foundation_pose':[
      'mycpp/build/*'
    ]},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mamad',
    maintainer_email='mamad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'foundationpose_ros_multi = foundation_pose.foundationpose_ros_multi:main',
          'foundation_pose = foundation_pose.foundation_pose_node:main',
          # used for testing
          'fake_realsense = foundation_pose.fake_realsense_node:main',


        ],
    },
)
