
all: dir cmdline_args kill_thread periodic_file_writer linux_copy reference_change std_function fun_with_templates

SRC_PATH=./src
BIN_PATH=./bin
#CC=g++ -std=c++11  # C++11
CC=g++ -std=c++1z  # C++17
#CC=arm-linux-gnueabi-g++ -std=c++1z  # C++17
#CC=/opt/Xilinx/SDK/2017.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++ -std=c++1z  # C++17

dir:
	mkdir -p $(BIN_PATH)

log4cxx:
	$(CC) $(SRC_PATH)/log4cxx.cpp -o $(BIN_PATH)/log4cxx -llog4cxx -lapr-1 -laprutil-1
	cp $(SRC_PATH)/Log4cxxConfig.xml $(BIN_PATH)

kill_thread:
	$(CC) $(SRC_PATH)/kill_thread.cpp -o $(BIN_PATH)/kill_thread

cmdline_args:
	$(CC) $(SRC_PATH)/cmdline_args.cpp -o $(BIN_PATH)/cmdline_args

periodic_file_writer:
	$(CC) $(SRC_PATH)/periodic_file_writer.cpp -o $(BIN_PATH)/periodic_file_writer

linux_copy:
	$(CC) $(SRC_PATH)/linux_copy.cpp -o $(BIN_PATH)/linux_copy -lstdc++fs

reference_change:
	$(CC) $(SRC_PATH)/reference_change.cpp -o $(BIN_PATH)/reference_change

std_function:
	$(CC) $(SRC_PATH)/std_function.cpp -o $(BIN_PATH)/std_function

fun_with_templates:
	$(CC) $(SRC_PATH)/fun_with_templates.cpp -o $(BIN_PATH)/fun_with_templates

clean:
	rm -rf $(BIN_PATH)

