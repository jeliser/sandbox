# This is a pseudo-Makefile where the make targets are really just wrappers around CMake commands.
# It allows the software system to be built without having to know the CMake (if you don't want
# to).  Additional targets (with cross compiling toolchains) will be added to the target list.

# This is not intended to replace using the real CMakes, but makes the typical case easier to run.

PREFIX?=build

# The cmake executable (try for cmake3 first, then default to cmake).  Cleaner for the CentOS machines
CMAKE?=$(shell which cmake3 || which cmake)
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
# The filter to determine which test binaries to execute
TEST_EXE?=*_test*
# The gtest filter option to choose which tests to execute
TEST_FILTER?="*"
# The gtest option to repeat the unit tests
TEST_REPEAT?=1
# The gtest option to repeat the unit tests
TEST_SHUFFLE?="--gtest_shuffle"
# The gtest option for the seed value
TEST_SEED?=0
# The gtest additional options
TEST_ARGS?=
# Build option for the generated library type (STATIC, SHARED)
LIB_TYPE?=SHARED
# Build the docs
DOCS?=OFF

default: all

# Generate the output directory using a method
define generate_build_dir
"${PREFIX}/${OS}-$(1)-${ARCH}-${TYPE}"
endef

# Build the TEST_ARGS command line options
GTEST_ARGS=--gtest_repeat=${TEST_REPEAT} --gtest_random_seed=${TEST_SEED} --gtest_filter=${TEST_FILTER} ${TEST_SHUFFLE} ${TEST_ARGS}

# Build the full set of options to be passed into cmake
MAKE_OPTIONS?=--no-print-directory
TOOLCHAIN_OPTIONS=-DCMAKE_TOOLCHAIN_FILE=.cmake/toolchain/${OS}-${COMPILER}-${ARCH}.cmake
BUILD_OPTIONS=-DCMAKE_MAKE_PROGRAM=make -DTESTS=${TESTS} -DDOCS=${DOCS} \
						  -DTEST_ARGS="${GTEST_ARGS}" -DLIB_TYPE=${LIB_TYPE} -G "Unix Makefiles"
							# -DARCH=${ARCH} -DTYPE=${TYPE} 

## The cmake 2-stage build process
preprocess::
	${CMAKE} -B$(call generate_build_dir,${COMPILER}) ${TOOLCHAIN_OPTIONS} ${BUILD_OPTIONS}
build:: preprocess
	${CMAKE} --build $(call generate_build_dir,${COMPILER}) --target ${TARGET} -- ${MAKE_OPTIONS} -j ${CORES}

## The default target
all:: TARGET=all
all:: build

## The debug target
debug:: TYPE=debug
debug:: all

## The release target
release:: TYPE=release
release:: all

## The test targets
test:: TESTS=ON
test:: all
tests:: test

## The test execution target
run-test:: run-tests
run-tests::TESTS=ON
run-tests::TARGET=run-tests
run-tests::build

## The code coverage target
coverage:: TESTS=ON
coverage:: TARGET=coverage
coverage:: build

## The valgrind target
valgrind:: TESTS=ON
valgrind:: TARGET=valgrind
valgrind:: build

## The static analyzer target
static-analysis::
	# Generate the Makefiles from the current source tree using Wrapping
	scan-build -k cmake -H. -B$(call generate_build_dir,$@) -DCMAKE_TOOLCHAIN_FILE=.cmake/toolchain/$@.cmake
	cd $(call generate_build_dir,$@) && scan-build -k -o ./results make -j ${CORES}

## The documentation target
docs:: DOCS=ON
docs:: TARGET=docs
docs:: build

## The clean targets
clean:: TARGET=clean
clean:: build

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
