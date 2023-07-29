CC = gcc
CFLAGS = -Wall

# List of source files
SRC_FILES = main.c rmlib.c

# List of object files (automatically generated from source files)
OBJ_FILES = $(SRC_FILES:.c=.o)

# The target executable
TARGET = main

all: $(TARGET)

$(TARGET): $(OBJ_FILES)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(OBJ_FILES) $(TARGET)
