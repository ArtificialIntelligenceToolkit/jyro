export NAME=`python3 setup.py --name 2>/dev/null`
export VERSION=`python3 setup.py --version 2>/dev/null`

all:
	rm -rf dist
	pip install wheel -U
	python3 setup.py register
	python3 setup.py bdist_wheel
	python3 setup.py sdist --formats=zip
	twine upload dist/*

tag:
	echo "Tagging $(NAME) $(VERSION)..." 
	git commit -a -m "Release $(VERSION)"; true
	git tag $(VERSION)
	git push --all
	git push --tags
