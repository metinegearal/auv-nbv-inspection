from setuptools import find_packages, setup
setup(name='auv_experiments', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_experiments']),
                  ('share/auv_experiments', ['package.xml'])],
      install_requires=['setuptools', 'optuna', 'PyYAML', 'pandas'], zip_safe=True,
      entry_points={'console_scripts': ['optuna_optimizer = auv_experiments.optuna_optimizer:main']},
      description='Outer AUV experiment orchestration', license='TODO: License declaration')
