from pathlib import Path
from setuptools import setup

PACKAGE = 'wafer_transport_sim'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
    ('share/' + PACKAGE, ['package.xml']),
]
for directory in ('launch', 'worlds', 'models', 'config', 'textures', 'scripts'):
    for parent in sorted({p.parent for p in Path(directory).rglob('*') if p.is_file()}):
        files = [str(p) for p in sorted(parent.iterdir())
                 if p.is_file() and p.suffix != '.pyc']
        if files and '__pycache__' not in str(parent):
            data_files.append(('share/' + PACKAGE + '/' + str(parent), files))

setup(
    name=PACKAGE, version='0.1.0', packages=[PACKAGE],
    data_files=data_files, install_requires=['setuptools'], extras_require={'test': ['pytest']},
    zip_safe=False,
    maintainer='Wafer Transport Demo maintainers',
    maintainer_email='maintainer@example.com',
    description='ROS 2 wafer handling and visual transport simulation',
    license='Apache-2.0', url='https://github.com/srigan-s/Wafer-Transport-Demo',
    entry_points={'console_scripts': [
        f'{name} = {PACKAGE}.{name}:main' for name in (
            'system_manager', 'wafer_handler', 'line_follower', 'ir_line_follower', 'qr_detector',
            'apriltag_detector', 'process_cell_controller',
            'transport_controller', 'docking_controller', 'integration_check',
            'four_room_controller', 'four_room_check')
    ]},
)
