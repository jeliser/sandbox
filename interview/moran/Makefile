
SRC_PATH=src
BIN_PATH=bin
CC=g++ -std=c++14 -g

all:: docker

docker::
	docker build -t interview-cpp:latest .

release::
	mkdir -p $(BIN_PATH)
	$(CC) -I$(SRC_PATH) $(SRC_PATH)/*.cpp -o $(BIN_PATH)/$@

test::
	mkdir -p $(BIN_PATH)
	$(CC) -I$(SRC_PATH) src/moran.cpp test/*.cpp -lgtest -lpthread -o $(BIN_PATH)/$@

.PHONY:: clean
clean::
	rm -rf $(BIN_PATH)

