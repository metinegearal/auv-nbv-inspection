from setuptools import find_packages, setup

package_name = 'auv_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'open3d', 'pyoctomap', 'matplotlib'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'live_viewer = auv_benchmark.live_viewer:main',
            'run_baselines = auv_benchmark.run_baselines:main',
            'plot_benchmark_results = auv_benchmark.plot_results:main',
            'save_planner_bans = auv_benchmark.adapters.permanent_bans:main',
        ],
    },
)
