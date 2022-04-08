# This is a pseudo-Makefile where the make targets are really just wrappers around CMake commands.
# It allows the software system to be built without having to know the CMake (if you don't want
# to).  Additional targets (with cross compiling toolchains) will be added to the target list.

# This is not intended to replace using the real CMakes, but makes the typical case easier to run.

PREFIX?=build

# Get the current system architecture if we are not cross-compiling
ARCH?=$(shell uname -p)
# Get the current system OS if we are not cross-compiling
OS?=$(shell echo $(shell uname) | tr A-Z a-z)
# Which compiler would you like to use?
COMPILER?=gcc
# Get the build type debug/release
TYPE?=debug
# The number of cores to build from
CORES?=8

# Build the tests
TESTS?=OFF
# Build the docs
DOCS?=OFF

# Generate the output directory using a method
define generate_build_dir
"${PREFIX}/${OS}-${ARCH}-${TYPE}"
endef

## The default target
all::
	cmake -H. -B$(call generate_build_dir,${COMPILER}) -DCMAKE_TOOLCHAIN_FILE=.cmake/toolchain/${OS}-${COMPILER}-${ARCH}.cmake -DTESTS=${TESTS} -DDOCS=${DOCS} && cmake --build $(call generate_build_dir,${COMPILER}) -- -j ${CORES}

## The debug target
debug:: TYPE=debug
debug:: all

## The release target
release:: TYPE=release
release:: TRACE=none
release:: all

## The test targets
test:: TESTS=ON
test:: all
tests:: test

## The test execution target
run-test: test
run-tests: run-test
	cmake --build $(call generate_build_dir,${COMPILER}) --target run-tests

## The code coverage target
coverage:: tests
	cmake --build $(call generate_build_dir,${COMPILER}) --target coverage

## The valgrind target
valgrind:: tests
	cmake --build $(call generate_build_dir,${COMPILER}) --target valgrind

## The static analyzer target
static-analysis::
	# Generate the Makefiles from the current source tree using Wrapping
	scan-build -k cmake -H. -B$(call generate_build_dir,$@) -DCMAKE_TOOLCHAIN_FILE=.cmake/toolchain/$@.cmake
	cd $(call generate_build_dir,$@) && scan-build -k -o ./results make -j ${CORES}

## The clean targets
clean::
	# Run clean from the generated Makefiles.
	cmake --build ${BUILD_DIR} --target clean

clean-all::
	@echo "Removing '${PREFIX}'"
	@rm -rf ${PREFIX}

## The format the code targets
format:: clang-format

clang-format::
	@echo "Formatting the source code"
	@find ./applications -iname *.h -o -iname *.cpp | xargs clang-format -i 
	@find ./common -iname *.h -o -iname *.cpp | xargs clang-format -i 
	@find ./test -iname *.h -o -iname *.cpp | xargs clang-format -i 

docker::
	docker build -t cmake:latest -f .Dockerfile .
