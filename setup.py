from setuptools import setup

package_name = 'husky_navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'utm'],
    zip_safe=True,
    maintainer='Safwan',
    maintainer_email='safwan@todo.todo',
    description='Husky navigation package with GPS-based navigation',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_navigation = husky_navigation_pkg.gps_navigation:main',
            'simple_gps_navigation = husky_navigation_pkg.simple_gps_navigation:main',
            'map_plot = husky_navigation_pkg.map_plot:main',
            'map_plot_v2 = husky_navigation_pkg.map_plot_v2:main',
        ],
    },
)

