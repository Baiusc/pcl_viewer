CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu99
LDFLAGS ?=
TARGET   = lidar_agent

all: $(TARGET)

$(TARGET): lidar_agent.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
