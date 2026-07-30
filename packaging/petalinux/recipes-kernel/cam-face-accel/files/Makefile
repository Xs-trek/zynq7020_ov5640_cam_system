obj-m := cam-face-accel.o

SRC := $(shell pwd)

all modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
