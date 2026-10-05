from setuptools import find_packages, setup
setup(name='telemetry_adapters', version='0.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/telemetry_adapters']),
                  ('share/telemetry_adapters', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      description='File-backed AUV telemetry adapters', license='TODO: License declaration')
