export NAME=`python3 setup.py --name 2>/dev/null`
export VERSION=`python3 setup.py --version 2>/dev/null`

all:
	pip install wheel -U
	python setup.py register
	python setup.py bdist_wheel
	python setup.py sdist --formats=zip
	twine upload dist/*

tag:
	echo "Tagging $(NAME) $(VERSION)..." 
	git commit -a -m "Release $(VERSION)"; true
	git tag $(VERSION)
	git push --all
	git push --tags
