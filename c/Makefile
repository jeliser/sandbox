
all: dir hello_world

SRC_PATH="./src"
BIN_PATH="./bin"

dir:
	mkdir -p $(BIN_PATH)

hello_world:
	g++ $(SRC_PATH)/hello_world.c -o $(BIN_PATH)/hello_world

clean:
	rm -rf $(BIN_PATH)

