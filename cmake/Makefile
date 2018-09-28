# This is a pseudo-Makefile where the make targets are really just wrappers around CMake commands.
# It allows the software system to be built without having to know the CMake (if you don't want
# to).  Additional targets (with cross compiling toolchains) will be added to the target list.

# This is not intended to replace using the real CMakes, but make the typical case easier to run.

all::
	# Generate the Makefiles from the current source tree using Wrapping
	cmake -H. -Bbuild -DENABLE_TESTS=OFF && cmake --build build -- -j 8

test::
	# Generate the Makefiles from the current source tree using Wrapping
	cmake -H. -Bbuild -DENABLE_TESTS=ON && cmake --build build -- -j 8

clean::
	# Run clean from the generated Makefiles.
	cmake --build build --target clean

clang-format::
	@echo "Formatting the source code"
	@find ./applications -iname *.h -o -iname *.cpp | xargs clang-format -i 
	@find ./common -iname *.h -o -iname *.cpp | xargs clang-format -i 
	@find ./test -iname *.h -o -iname *.cpp | xargs clang-format -i 
