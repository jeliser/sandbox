
all: dir hello_world inotify filesize_total

SRC_PATH=./src
BIN_PATH=./bin
CC=gcc

dir:
	mkdir -p $(BIN_PATH)

hello_world:
	$(CC) $(SRC_PATH)/hello_world.c -o $(BIN_PATH)/hello_world

inotify:
	$(CC) $(SRC_PATH)/demo_inotify.c -o $(BIN_PATH)/inotify

filesize_total:
	$(CC) $(SRC_PATH)/prsize.c -o $(BIN_PATH)/filesize_total

clean:
	rm -rf $(BIN_PATH)

