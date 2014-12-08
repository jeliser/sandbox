
all: dir log4cxx kill_thread

SRC_PATH="./src"
BIN_PATH="./bin"

dir:
	mkdir -p $(BIN_PATH)

log4cxx:
	gcc $(SRC_PATH)/log4cxx.cpp -o $(BIN_PATH)/log4cxx -llog4cxx -lapr-1 -laprutil-1
	cp $(SRC_PATH)/Log4cxxConfig.xml $(BIN_PATH)

kill_thread:
	g++ $(SRC_PATH)/kill_thread.cpp -o $(BIN_PATH)/kill_thread

clean:
	rm -rf $(BIN_PATH)

