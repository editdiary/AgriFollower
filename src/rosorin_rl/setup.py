import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rosorin_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds_models'),
            glob(os.path.join('worlds_models', '*.sdf'))
            + glob(os.path.join('worlds_models', '*.png'))),  # AprilTag 텍스처 포함
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhlee',
    maintainer_email='dev@example.com',
    description='ROSOrin 작업자 추종 강화학습 패키지 (Gymnasium Env + SB3).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run rosorin_rl <이름> 으로 실행되는 노드/스크립트들
            'target_controller = rosorin_rl.target_controller_node:main',
            'target_feature = rosorin_rl.target_feature_node:main',
            'train_sac = rosorin_rl.train_sac:main',
            'eval_policy = rosorin_rl.eval_policy:main',
            'eval_sweep = rosorin_rl.eval_sweep:main',
            'analyze_log = rosorin_rl.analyze_log:main',
        ],
    },
)
