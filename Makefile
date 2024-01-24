.PHONY: all clean cwb vitis

all:
	echo "Target is not specified."
	exit 1

cwb:
	$(MAKE) -C ros2rapper -f Makefile.cwb synth

vitis:
	$(MAKE) -C ros2rapper -f Makefile.vitis synth

clean: clean
	$(MAKE) -C ros2rapper -f Makefile.vitis clean
	$(MAKE) -C ros2rapper -f Makefile.cwb clean
