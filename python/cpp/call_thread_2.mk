call_thread_2: 	call_thread_2.o
		
	gcc -o call_thread_2 call_thread_2.o -export-dynamic -L/usr/local/lib/python2.4/config -lpython2.4 -lpthread -lm -ldl -lutil


call_thread_2.o: call_thread_2.c	
		
	gcc -c call_thread_2.c -I/usr/local/include/python2.4 


clean:		
		
	rm call_thread_2.o call_thread_2
