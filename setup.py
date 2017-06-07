import io

try:
    from setuptools.core import setup
except ImportError:
    from distutils.core import setup
import sys

svem_flag = '--single-version-externally-managed'
if svem_flag in sys.argv:
    # Die, setuptools, die.
    sys.argv.remove(svem_flag)

with io.open('jyro/__init__.py', encoding='utf-8') as fid:
    for line in fid:
        if line.startswith('__version__'):
            version = line.strip().split()[-1][1:-1]
            break

setup(name='jyro',
      version=version,
      description='Python tools for robots',
      long_description="Python and Jupyter tools for robots",
      author='Douglas Blank',
      author_email='doug.blank@gmail.com',
      url="https://github.com/Calysto/jyro",
      install_requires=['calysto'],
      packages=[
          'jyro',
          'jyro.simulator',
          'jyro.myro',
          ],
      classifiers = [
          'Framework :: IPython',
          'License :: OSI Approved :: BSD License',
          'Programming Language :: Python :: 3',
      ]
)
