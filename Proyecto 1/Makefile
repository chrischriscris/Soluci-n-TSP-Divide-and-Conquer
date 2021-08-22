KC= kotlinc
KFLAGS= -include-runtime
SRC= DACTSP.kt
PROG= DACTSP.jar

all:
	$(KC) $(SRC) $(KFLAGS) -d $(PROG)

.PHONY: clean

clean:
	rm -rf $(PROG)
