call_class: 	call_class.o
	g++ -g -o call_class call_class.o -L/usr/lib/python3.7/config-3.7m-x86_64-linux-gnu -lpython3.7 -lpthread -lm -ldl -lutil

call_class.o:	call_class.cpp
	g++ -g -c call_class.cpp -I/usr/include/python3.7m

clean:		
	rm call_class.o call_class
