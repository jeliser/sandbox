
all: dir cmdline_args kill_thread periodic_file_writer

SRC_PATH=./src
BIN_PATH=./bin
CC="g++"

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
	$(CC) $(SRC_PATH)/periodic_file_writer.cpp -o $(BIN_PATH)/periodic_file_writer -std=c++11

clean:
	rm -rf $(BIN_PATH)

