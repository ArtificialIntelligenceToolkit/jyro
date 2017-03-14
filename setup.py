try:
    from setuptools.core import setup
except ImportError:
    from distutils.core import setup
import sys

svem_flag = '--single-version-externally-managed'
if svem_flag in sys.argv:
    # Die, setuptools, die.
    sys.argv.remove(svem_flag)

setup(name='jyro',
      version='0.1.0',
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
