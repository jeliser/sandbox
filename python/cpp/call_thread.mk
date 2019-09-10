call_thread: 	call_thread.o
		
	gcc -o call_thread call_thread.o -export-dynamic -L/usr/local/lib/python2.4/config -lpython2.4 -lpthread -lm -ldl -lutil


call_thread.o:	call_thread.c	
		
	gcc -c call_thread.c -I/usr/local/include/python2.4 


clean:		
		
	rm call_thread.o call_thread
