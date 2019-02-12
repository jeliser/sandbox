
all: dir hello_world inotify filesize_total mosquitto

SRC_PATH=./src
BIN_PATH=./bin
CC=gcc

dir:
	mkdir -p $(BIN_PATH)

mosquitto: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@ -lmosquitto

.DEFAULT: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@

clean:
	rm -rf $(BIN_PATH)

