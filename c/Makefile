
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

CODE_FORMAT_FILES=-iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.icc" -o -iname "*.c"

format::
	@echo "Formatting the source code"
	@find . ${CODE_FORMAT_FILES} | xargs clang-format -style=file -i -sort-includes=false

docker::
	docker build -t c:latest -f .Dockerfile .
