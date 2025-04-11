OBJS := main.c
TARGET := main
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $(TARGET) $(CFLAGS) $(OBJS) $(LDFLAGS) -lperiphery


$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all clean