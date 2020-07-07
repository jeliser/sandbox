
all: cmdline_args kill_thread periodic_file_writer linux_copy reference_change std_function fun_with_templates dlopen_example thread_pids consume_cpu select timer code_coverage floating_point_precision ws_server ws_client custom_deleter any time_conversion

SRC_PATH=./src
BIN_PATH=./bin
#CC=g++ -std=c++11 -g # C++11
CC=g++ -std=c++14 -g # C++14
#CC=g++ -std=c++1z -g # C++17
#CC=arm-linux-gnueabi-g++ -std=c++1z  # C++17
#CC=/opt/Xilinx/SDK/2017.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++ -std=c++1z  # C++17

dir:
	mkdir -p $(BIN_PATH)

code_coverage: dir
	$(CC) $(SRC_PATH)/pointers.cpp -o $(BIN_PATH)/code_coverage -fprofile-arcs -ftest-coverage

log4cxx: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -llog4cxx -lapr-1 -laprutil-1
	cp $(SRC_PATH)/Log4cxxConfig.xml $(BIN_PATH)

linux_copy: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -lstdc++fs

dlopen_example: dir
	$(CC) $(SRC_PATH)/dso_library_file.cpp -o $(BIN_PATH)/dso_library_file.so -shared -fPIC
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -ldl -rdynamic

consume_cpu: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -lpthread

# sudo apt install libwebsockets-dev 
ws_server: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -lpthread -lwebsockets -Wno-deprecated-declarations

# sudo apt install libwebsockets-dev 
ws_client: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -lpthread -lwebsockets -Wno-deprecated-declarations -Wno-write-strings

proto:
	protoc src/Record01.proto --cpp_out ./
	# -lpthread needs to be included with the latest libprotobuf some reason
	$(CC) -I./ $(SRC_PATH)/Record01.pb.cc $(SRC_PATH)/proto_example.cpp -o $(BIN_PATH)/$@ -lprotobuf -lpthread

.DEFAULT: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@

clean:
	rm -rf $(BIN_PATH)

CODE_FORMAT_FILES=-iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.icc" -o -iname "*.c"

format::
	@echo "Formatting the source code"
	@find . ${CODE_FORMAT_FILES} | xargs clang-format -style=file -i -sort-includes=false
