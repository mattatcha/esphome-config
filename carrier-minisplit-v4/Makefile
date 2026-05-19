CONFIG_PATH = carrier-minisplit.yaml
ARGS = # --device 192.168.2.214
all: compile upload logs

.PHONY: all compile upload logs

compile:
	esphome compile $(CONFIG_PATH)

upload:
	esphome upload $(CONFIG_PATH) $(ARGS)

logs:
	esphome logs $(CONFIG_PATH) $(ARGS)
