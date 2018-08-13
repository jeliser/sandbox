
all: cmdline_args kill_thread periodic_file_writer linux_copy reference_change std_function fun_with_templates dlopen_example thread_pids consume_cpu

SRC_PATH=./src
BIN_PATH=./bin
#CC=g++ -std=c++11  # C++11
CC=g++ -std=c++1z  # C++17
#CC=arm-linux-gnueabi-g++ -std=c++1z  # C++17
#CC=/opt/Xilinx/SDK/2017.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++ -std=c++1z  # C++17

dir:
	mkdir -p $(BIN_PATH)

log4cxx: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -llog4cxx -lapr-1 -laprutil-1
	cp $(SRC_PATH)/Log4cxxConfig.xml $(BIN_PATH)

linux_copy: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -lstdc++fs

dlopen_example: dir
	$(CC) $(SRC_PATH)/dso_library_file.cpp -o $(BIN_PATH)/dso_library_file.so -shared -fPIC
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -ldl -rdynamic

consume_cpu: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@ -pthread

.DEFAULT: dir
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@

clean:
	rm -rf $(BIN_PATH)

