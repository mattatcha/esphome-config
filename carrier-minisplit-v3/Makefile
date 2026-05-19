CONFIG_PATH = carrier-minisplit.yaml
ARGS = # --device 192.168.2.181
all: compile upload logs

.PHONY: all

compile:
	esphome compile $(CONFIG_PATH)

.PHONY: compile

upload:
	esphome upload $(CONFIG_PATH) $(ARGS)

.PHONY: upload

logs:
	esphome logs $(CONFIG_PATH) $(ARGS)

.PHONY: logs
