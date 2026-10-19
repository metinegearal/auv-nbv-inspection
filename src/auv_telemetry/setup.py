from setuptools import find_packages, setup
setup(name='auv_telemetry', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/auv_telemetry']),
                  ('share/auv_telemetry', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      description='File-backed AUV telemetry adapters', license='Apache-2.0')
