from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = 'perception'


class RuntimeBuild(build_py):
    """Keep checkout-only research runners outside the inference distribution."""

    def find_package_modules(self, package, package_dir):
        modules = super().find_package_modules(package, package_dir)
        research = {'predict', 'predict_sample', 'prediction_registry'}
        return [module for module in modules
                if not (package == 'perception.BGNet' and module[1] in research)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', '*.datasets', '*.dataloader']),
    package_data={'perception.BGNet': ['models/Sceneflow-IRS-BGNet-Plus.pth']},
    cmdclass={'build_py': RuntimeBuild},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'torch', 'torchvision', 'Pillow'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_estimation = perception.depth_estimation:main'
        ],
    },
)
