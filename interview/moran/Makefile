
SRC_PATH=./src
BIN_PATH=./bin
CC=g++ -std=c++14 -g

all:: docker

.DEFAULT::
	mkdir -p $(BIN_PATH)
	$(CC) -L$(SRC_PATH) $(SRC_PATH)/*.cpp -o $(BIN_PATH)/$@

test::
	mkdir -p $(BIN_PATH)
	g++ src/moran.cpp test/main.cpp -lgtest -lpthread -o $(BIN_PATH)/$@

docker::
	docker build -t interview-cpp:latest .

.PHONY:: clean
clean::
	rm -rf $(BIN_PATH)

