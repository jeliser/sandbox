call_function: 	call_function.o
	g++ -g -o call_function call_function.o -export-dynamic -L/usr/lib/python3.7/config-3.7m-x86_64-linux-gnu -lpython3.7 -lpthread -lm -ldl -lutil

call_function.o:call_function.cpp
	g++ -g -c call_function.cpp -I/usr/include/python3.7

clean:		
	rm call_function.o call_function
