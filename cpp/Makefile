
all: dir log4cxx

SRC_PATH="./src"
BIN_PATH="./bin"

dir:
	mkdir -p $(BIN_PATH)

log4cxx:
	gcc $(SRC_PATH)/log4cxx.cpp -o $(BIN_PATH)/log4cxx -llog4cxx -lapr-1 -laprutil-1

clean:
	rm -rf $(BIN_PATH)

