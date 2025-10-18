include ../module.mk

test: test.c history.c
	$(CC) -g -Wall $^ -o test -L.. -lmisc -lm -Wl,-rpath,..

clean: clean_test

clean_test:
	-rm -rf test log_*
