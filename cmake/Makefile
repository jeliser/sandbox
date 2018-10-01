# This is a pseudo-Makefile where the make targets are really just wrappers around CMake commands.
# It allows the software system to be built without having to know the CMake (if you don't want
# to).  Additional targets (with cross compiling toolchains) will be added to the target list.

# This is not intended to replace using the real CMakes, but makes the typical case easier to run.

PREFIX=build

# Get the current system architecture if we are not cross-compiling
ifndef ARCH
	ARCH=$(shell uname -p)
endif
# Get the current system OS if we are not cross-compiling
ifndef OS
	OS=$(shell echo $(shell uname) | tr A-Z a-z)
endif
# Get the build type (default: debug)
ifndef TYPE
	TYPE=debug
endif

# Generate the output directory
ifndef BUILD_DIR
	BUILD_DIR="${PREFIX}/${OS}-${ARCH}-${TYPE}"
endif

# The number of cores to build from
ifndef CORES
	CORES=8
endif

## The default target
all::
	# Generate the Makefiles from the current source tree using Wrapping
	cmake -H. -B${BUILD_DIR} -DCMAKE_TOOLCHAIN_FILE=toolchain/${OS}-${ARCH}.cmake -DENABLE_TESTS=OFF && cmake --build ${BUILD_DIR} -- -j ${CORES}

## The test target
test::
	# Generate the Makefiles from the current source tree using Wrapping
	cmake -H. -B${BUILD_DIR} -DENABLE_TESTS=ON && cmake --build ${BUILD_DIR} -- -j ${CORES}

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
