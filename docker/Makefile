COMPOSE = docker compose
SERVICE ?= dev

.PHONY: help up shell run down stop build logs ps restart

help:
	@echo "Targets:"
	@echo "  up      - start $(SERVICE) in background and exec into shell"
	@echo "  shell   - exec into running container"
	@echo "  run     - ephemeral container shell (separate container)"
	@echo "  build   - build image(s)"
	@echo "  down    - stop and remove containers"
	@echo "  stop    - stop containers"
	@echo "  logs    - follow logs for $(SERVICE)"
	@echo "  ps      - list compose services"
	@echo "  restart - restart $(SERVICE)"

up:
	$(COMPOSE) up -d $(SERVICE)
	$(COMPOSE) exec $(SERVICE) bash

shell:
	$(COMPOSE) exec $(SERVICE) bash

run:
	$(COMPOSE) run --rm --service-ports $(SERVICE) bash

build:
	$(COMPOSE) build $(SERVICE)

down:
	$(COMPOSE) down

stop:
	$(COMPOSE) stop

logs:
	$(COMPOSE) logs -f $(SERVICE)

ps:
	$(COMPOSE) ps

restart:
	$(COMPOSE) restart $(SERVICE)

