all:
	make -f call_class.mk
	make -f call_function.mk
	#make -f call_thread.mk
	#make -f call_thread_2.mk

clean:
	rm -f *.o *.pyc call_class call_function call_thread call_thread_2

CODE_FORMAT_FILES=-iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.icc" -o -iname "*.c"

format::
	@echo "Formatting the source code"
	@find . ${CODE_FORMAT_FILES} | xargs clang-format -style=file -i -sort-includes=false
