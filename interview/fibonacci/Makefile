
SRC_PATH=./src
BIN_PATH=./bin
CC=g++ -std=c++14 -g

all:: docker

.DEFAULT::
	mkdir -p $(BIN_PATH)
	$(CC) $(SRC_PATH)/$@.cpp -o $(BIN_PATH)/$@

docker::
	docker build -t interview-cpp:latest .

.PHONY:: dir
dir::
	mkdir -p $(BIN_PATH)

.PHONY:: clean
clean::
	rm -rf $(BIN_PATH)

