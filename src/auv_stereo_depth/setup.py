from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = 'auv_stereo_depth'


class RuntimeBuild(build_py):
    """Keep checkout-only research runners outside the inference distribution."""

    def find_package_modules(self, package, package_dir):
        modules = super().find_package_modules(package, package_dir)
        research = {'predict', 'predict_sample', 'prediction_registry'}
        return [module for module in modules
                if not (package == 'auv_stereo_depth.BGNet' and module[1] in research)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', '*.datasets', '*.dataloader']),
    package_data={'auv_stereo_depth.BGNet': ['models/Sceneflow-IRS-BGNet-Plus.pth']},
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
    description='Stereo depth estimation; Apache-2.0 covers project code only. BGNet/model terms remain unverified; see the repository-root THIRD_PARTY_NOTICES.md.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_estimation = auv_stereo_depth.depth_estimation:main'
        ],
    },
)
