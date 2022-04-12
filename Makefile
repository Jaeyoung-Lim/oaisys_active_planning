package?=oaisys_client

format:
	Tools/fix_code_style.sh .

build:
	catkin build ${package}

build-test:
	catkin build ${package} --no-deps -i --catkin-make-args tests
