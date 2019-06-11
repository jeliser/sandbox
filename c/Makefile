
all: dir hello_world inotify filesize_total mosquitto azure_event_hub ws_server ws_client

SRC_PATH=./src
BIN_PATH=./bin
CC=gcc

dir:
	mkdir -p $(BIN_PATH)

azure_event_hub: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@ -lqpid-proton

mosquitto: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@ -lmosquitto

# sudo apt install libwebsockets-dev 
ws_server: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@ -lpthread -lwebsockets -Wno-deprecated-declarations

# sudo apt install libwebsockets-dev 
ws_client: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@ -lpthread -lwebsockets -Wno-deprecated-declarations -Wno-write-strings


.DEFAULT: dir
	$(CC) $(SRC_PATH)/$@.c -o $(BIN_PATH)/$@

clean:
	rm -rf $(BIN_PATH)

