
all: dir hello_world inotify

SRC_PATH=./src
BIN_PATH=./bin
CC=gcc

dir:
	mkdir -p $(BIN_PATH)

hello_world:
	$(CC) $(SRC_PATH)/hello_world.c -o $(BIN_PATH)/hello_world

inotify:
	$(CC) $(SRC_PATH)/demo_inotify.c -o $(BIN_PATH)/inotify

clean:
	rm -rf $(BIN_PATH)

