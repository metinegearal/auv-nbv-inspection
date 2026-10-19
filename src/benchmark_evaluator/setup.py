from setuptools import find_packages, setup

package_name = 'benchmark_evaluator'

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
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_baselines = benchmark_evaluator.run_baselines:main',
            'plot_benchmark_results = benchmark_evaluator.plot_results:main',
            'save_planner_bans = benchmark_evaluator.adapters.permanent_bans:main',
        ],
    },
)
