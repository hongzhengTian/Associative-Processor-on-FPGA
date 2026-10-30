# Top-level targets.
#
#   make test     build the software, run all simulations
#   make synth    Yosys synthesis for UltraScale+, prints a resource summary
#   make lint     Verilator lint of the RTL
#   make clean

.PHONY: all test sw sim synth lint clean
all: test

test: sim

sw:
	$(MAKE) -C sw

sim:
	$(MAKE) -C sim all

synth:
	$(MAKE) -C syn all

lint:
	$(MAKE) -C sim lint

clean:
	$(MAKE) -C sw clean
	$(MAKE) -C sim clean
	$(MAKE) -C syn clean
