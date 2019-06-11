
all: interface shapes messages gcp_hello_world

SRC_PATH=./src
BIN_PATH=./bin
CC=go

dir:
	mkdir -p $(BIN_PATH)

.DEFAULT: dir
	$(CC) build -o $(BIN_PATH)/$@ $(SRC_PATH)/$@.go

clean:
	rm -rf $(BIN_PATH)

