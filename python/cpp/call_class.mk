all:
	g++ -g -fPIC -c call_class.cpp -o call_class.o -I/usr/include/python3.7m
	g++ -g -shared -o libcall_class.so call_class.o -L/usr/lib/python3.7/config-3.7m-x86_64-linux-gnu -lpython3.7
	g++ -g run_call_class.cpp -o run_me -lpthread -lm -ldl -lutil

clean:		
	rm call_class.so call_class
