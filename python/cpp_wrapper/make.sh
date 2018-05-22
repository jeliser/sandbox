gcc -fpic --shared $( python-config --includes ) cutils/utilsmodule.c -o cutils/utilsmodule.so
