from pathlib import Path
from setuptools import setup

PACKAGE = 'raspbot_chassis'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
    ('share/' + PACKAGE, ['package.xml']),
]
for directory in ('launch', 'worlds', 'models', 'config'):
    for parent in sorted({p.parent for p in Path(directory).rglob('*') if p.is_file()}):
        files = [str(p) for p in sorted(parent.iterdir())
                 if p.is_file() and p.suffix != '.pyc']
        if files and '__pycache__' not in str(parent):
            data_files.append(('share/' + PACKAGE + '/' + str(parent), files))

setup(
    name=PACKAGE, version='0.1.0', packages=[],
    data_files=data_files, install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Wafer Transport Demo maintainers',
    maintainer_email='maintainer@example.com',
    description='Minimal Raspbot V2 chassis simulation',
    license='Apache-2.0', url='https://github.com/srigan-s/Wafer-Transport-Demo',
    entry_points={'console_scripts': []},
)
