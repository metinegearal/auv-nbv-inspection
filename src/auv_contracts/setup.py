from setuptools import find_packages, setup
setup(name='auv_contracts', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_contracts']),
                  ('share/auv_contracts', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      description='Framework-independent AUV values and client-owned boundary contracts', license='Apache-2.0')
