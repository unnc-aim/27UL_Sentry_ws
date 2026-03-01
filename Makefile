SUDO ?= sudo
SYSTEMCTL ?= systemctl
JOURNALCTL ?= journalctl

SERVICES := ros2-foxglove-bridge.service ros2-soem-bringup.service ros2-infantry-chassis.service
SERVICE ?= ros2-infantry-chassis.service

.PHONY: help
.PHONY: status status-all start start-all stop stop-all restart restart-all
.PHONY: enable enable-all disable disable-all logs daemon-reload

help:
	@echo "Usage: make <target> [SERVICE=<service-name>]"
	@echo ""
	@echo "Single service targets (default SERVICE=$(SERVICE))"
	@echo "  make status"
	@echo "  make start"
	@echo "  make stop"
	@echo "  make restart"
	@echo "  make enable"
	@echo "  make disable"
	@echo "  make logs SERVICE=ros2-soem-bringup.service"
	@echo ""
	@echo "All services targets"
	@echo "  make status-all"
	@echo "  make start-all"
	@echo "  make stop-all"
	@echo "  make restart-all"
	@echo "  make enable-all"
	@echo "  make disable-all"
	@echo ""
	@echo "Configured services:"
	@for s in $(SERVICES); do echo "  - $$s"; done

daemon-reload:
	$(SUDO) $(SYSTEMCTL) daemon-reload

status:
	$(SUDO) $(SYSTEMCTL) status $(SERVICE)

start:
	$(SUDO) $(SYSTEMCTL) start $(SERVICE)

stop:
	$(SUDO) $(SYSTEMCTL) stop $(SERVICE)

restart:
	$(SUDO) $(SYSTEMCTL) restart $(SERVICE)

enable:
	$(SUDO) $(SYSTEMCTL) enable $(SERVICE)

disable:
	$(SUDO) $(SYSTEMCTL) disable $(SERVICE)

logs:
	$(SUDO) $(JOURNALCTL) -u $(SERVICE) -f

status-all:
	$(SUDO) $(SYSTEMCTL) status $(SERVICES)

start-all:
	$(SUDO) $(SYSTEMCTL) start $(SERVICES)

stop-all:
	$(SUDO) $(SYSTEMCTL) stop $(SERVICES)

restart-all:
	$(SUDO) $(SYSTEMCTL) restart $(SERVICES)

enable-all:
	$(SUDO) $(SYSTEMCTL) enable $(SERVICES)

disable-all:
	$(SUDO) $(SYSTEMCTL) disable $(SERVICES)
