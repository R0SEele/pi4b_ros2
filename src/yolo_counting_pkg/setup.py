from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os  # 必须导入os模块
from glob import glob  # 必须导入glob模块


def file_glob(pattern):
    return [p for p in glob(pattern) if os.path.isfile(p)]

package_name = 'yolo_counting_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        # 安装模型/数据/结果目录
        (os.path.join('share', package_name, 'models'), file_glob('models/*')),
        (os.path.join('share', package_name, 'data/vedio'), file_glob('data/vedio/*')),
        (os.path.join('share', package_name, 'results'), file_glob('results/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rose',
    maintainer_email='rose@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_counting_pkg.yolo_detector:main',
            'priority_calc = yolo_counting_pkg.priority_calc:main',
        ],
    },
)
